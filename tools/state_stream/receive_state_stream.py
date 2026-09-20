#!/usr/bin/env python3
"""Independent, standard-library-only receiver for ORVD state_stream."""

from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import time
from collections import OrderedDict
from pathlib import Path

HEADER = struct.Struct("<4sHHQQdHHI")
BODY = struct.Struct("<13d")
CHUNK = 1200 - HEADER.size


class Receiver:
    """Bounded reassembly. Only whole, newer frames are released to consumers."""

    def __init__(self):
        self.pending = OrderedDict()
        self.latest = {}
        self.descriptions = {}
        self.statistics = {
            "datagrams": 0,
            "invalid_datagrams": 0,
            "duplicate_or_old_datagrams": 0,
            "incomplete_messages_evicted": 0,
            "state_frames": 0,
            "sequence_gaps": 0,
        }

    def feed(self, packet):
        self.statistics["datagrams"] += 1
        try:
            return self._feed(packet)
        except (ValueError, struct.error, UnicodeError, KeyError, TypeError):
            self.statistics["invalid_datagrams"] += 1
            return None

    def _feed(self, packet):
        if not HEADER.size <= len(packet) <= 1200:
            raise ValueError("packet length")
        magic, version, kind, run_id, seq, sim_time, part, parts, total = HEADER.unpack_from(packet)
        if magic != b"OSTS" or version != 1 or kind not in (1, 2) or not math.isfinite(sim_time):
            raise ValueError("header")
        if parts != max(1, (total + CHUNK - 1) // CHUNK) or part >= parts:
            raise ValueError("parts")
        payload = packet[HEADER.size :]
        if len(payload) != min(CHUNK, total - part * CHUNK):
            raise ValueError("part length")
        lane = (run_id, kind)
        if seq <= self.latest.get(lane, -1):
            self.statistics["duplicate_or_old_datagrams"] += 1
            return None
        key = (run_id, kind, seq)
        if key not in self.pending:
            self.pending[key] = (sim_time, parts, total, {})
            if len(self.pending) > 8:
                self.pending.popitem(last=False)
                self.statistics["incomplete_messages_evicted"] += 1
        prior_time, prior_parts, prior_total, chunks = self.pending[key]
        if (sim_time, parts, total) != (prior_time, prior_parts, prior_total):
            raise ValueError("conflicting fragments")
        if part in chunks:
            if chunks[part] != payload:
                raise ValueError("conflicting duplicate")
            self.statistics["duplicate_or_old_datagrams"] += 1
            return None
        chunks[part] = payload
        if len(chunks) != parts:
            return None
        del self.pending[key]
        payload = b"".join(chunks[index] for index in range(parts))
        result = {"run_id": run_id, "sequence": seq, "simulation_time_seconds": sim_time}
        if kind == 1:
            descriptor = json.loads(payload)
            if descriptor["schema"] != "orvd.state-stream":
                raise ValueError("descriptor")
            names = descriptor["body_names"]
            if not isinstance(names, list) or any(not isinstance(name, str) for name in names):
                raise ValueError("body names")
            fields = descriptor["scalars"]
            if not isinstance(fields, list) or any(not isinstance(f["name"], str) for f in fields):
                raise ValueError("scalar definitions")
            self.descriptions[run_id] = descriptor
            result.update(kind="description", description=descriptor)
        else:
            count = struct.unpack_from("<I", payload)[0]
            offset = 4 + count * BODY.size
            scalar_count = struct.unpack_from("<I", payload, offset)[0]
            if len(payload) != offset + 4 + scalar_count * 9:
                raise ValueError("state size")
            values = [list(BODY.unpack_from(payload, 4 + i * BODY.size)) for i in range(count)]
            if any(not math.isfinite(v) for row in values for v in row):
                raise ValueError("nonfinite state")
            scalars = list(struct.unpack_from(f"<{scalar_count}d", payload, offset + 4))
            statuses = list(payload[offset + 4 + scalar_count * 8 :])
            if any(
                not math.isfinite(v) or s not in (0, 1, 2) or (s != 1 and v != 0)
                for v, s in zip(scalars, statuses, strict=True)
            ):
                raise ValueError("scalar value/status")
            descriptor = self.descriptions.get(run_id)
            if descriptor is not None and (
                len(descriptor["body_names"]) != count or len(descriptor["scalars"]) != scalar_count
            ):
                raise ValueError("description count")
            previous = self.latest.get(lane)
            if previous is not None:
                self.statistics["sequence_gaps"] += seq - previous - 1
            self.statistics["state_frames"] += 1
            result.update(
                kind="state", bodies=values, scalar_values=scalars, scalar_statuses=statuses
            )
        self.latest[lane] = seq
        # Limit bookkeeping for receivers left open across many separate runs.
        if len(self.latest) > 16:
            oldest_run = next(iter(self.latest))[0]
            self.latest = {k: v for k, v in self.latest.items() if k[0] != oldest_run}
            self.descriptions.pop(oldest_run, None)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=10099)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--capture-states", action="store_true")
    parser.add_argument(
        "--idle-seconds",
        type=float,
        default=2.0,
        help="Finish after this quiet interval following the first datagram",
    )
    args = parser.parse_args()
    receiver = Receiver()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as channel:
        channel.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        channel.bind((args.bind, args.port))
        channel.settimeout(0.25)
        args.output_directory.mkdir(parents=True, exist_ok=False)
        print(json.dumps({"listening": channel.getsockname()}), flush=True)
        capture = (
            (args.output_directory / "states.jsonl").open("x") if args.capture_states else None
        )
        first = last = last_packet = None
        try:
            while True:
                try:
                    packet, _source = channel.recvfrom(65535)
                except TimeoutError:
                    if (
                        last_packet is not None
                        and time.monotonic() - last_packet >= args.idle_seconds
                    ):
                        break
                    continue
                last_packet = time.monotonic()
                decoded = receiver.feed(packet)
                if decoded is None:
                    continue
                if decoded["kind"] == "description":
                    destination = args.output_directory / f"description-{decoded['run_id']}.json"
                    destination.write_text(json.dumps(decoded, indent=2) + "\n")
                else:
                    first = decoded if first is None else first
                    last = decoded
                    if capture:
                        capture.write(json.dumps(decoded, allow_nan=False) + "\n")
        except KeyboardInterrupt:
            pass
        finally:
            if capture:
                capture.close()
        summary = {
            **receiver.statistics,
            "incomplete_messages_pending": len(receiver.pending),
            "first_state": first,
            "last_state": last,
        }
        (args.output_directory / "receiver_summary.json").write_text(
            json.dumps(summary, indent=2, allow_nan=False) + "\n"
        )
        print(json.dumps(receiver.statistics), flush=True)


if __name__ == "__main__":
    main()
