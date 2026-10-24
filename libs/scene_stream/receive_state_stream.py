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


def decode_description(payload):
    document = json.loads(payload, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
    if document["schema"] != "orvd.scene_description":
        raise ValueError("description schema")
    description = document["description"]
    for key, expected in (("length_unit", "meter"), ("time_unit", "second"),
                          ("angle_unit", "radian")):
        if description[key] != expected:
            raise ValueError("description unit")
    if description["orientation"] != (
            "unit quaternion [w, x, y, z] mapping body coordinates to world "
            "coordinates; a wheel body's orientation includes its spin"):
        raise ValueError("orientation convention")
    for key in ("world_frame", "orientation"):
        if not isinstance(description[key], str) or not description[key]:
            raise ValueError("description convention")
    period = description["nominal_sample_period_seconds"]
    if period is not None and (not isinstance(period, (int, float)) or isinstance(period, bool)
                               or not math.isfinite(period) or period <= 0):
        raise ValueError("nominal sample period")
    expected_status = {"0": "not_ready", "1": "valid", "2": "placeholder"}
    expected_phase = {"0": "initial_accepted_state", "1": "dense_intermediate_sample",
                      "2": "accepted_endpoint"}
    if (description["scalar_status_codes"] != expected_status or
            description["phase_codes"] != expected_phase):
        raise ValueError("description codes")
    bodies, scalars, wheels = (description[key] for key in ("bodies", "scalars", "wheel_placements"))
    if not all(isinstance(items, list) for items in (bodies, scalars, wheels)):
        raise ValueError("description arrays")
    def names(items, key):
        result = [item[key] for item in items]
        if any(not isinstance(value, str) or not value for value in result) or len(set(result)) != len(result):
            raise ValueError("description names")
        return result
    body_names = names(bodies, "name")
    if not body_names:
        raise ValueError("empty topology")
    names(wheels, "interface_name")
    scalar_names = names(scalars, "name")
    if any(not isinstance(body["moves_freely_in_world"], bool) for body in bodies):
        raise ValueError("body mobility")
    for field in scalars:
        if any(not isinstance(field[key], str) for key in
               ("unit", "reference_frame", "quantity", "method", "sample_semantics")):
            raise ValueError("scalar definition")
    carrier_sides = set()
    for wheel in wheels:
        if wheel["wheel_body_name"] not in body_names or wheel["carrier_body_name"] not in body_names:
            raise ValueError("wheel body reference")
        for key in ("datum_in_wheel_body_frame_meters", "spin_axis_in_wheel_body_frame"):
            vector = wheel[key]
            if not isinstance(vector, list) or len(vector) != 3 or any(
                    not isinstance(value, (int, float)) or isinstance(value, bool) or
                    not math.isfinite(value) for value in vector):
                raise ValueError("wheel vector")
        if abs(sum(value * value for value in wheel["spin_axis_in_wheel_body_frame"]) - 1.0) > 1e-8:
            raise ValueError("wheel axis")
        radius = wheel["nominal_rolling_radius_meters"]
        if not isinstance(radius, (int, float)) or isinstance(radius, bool) or not math.isfinite(radius) or radius <= 0:
            raise ValueError("wheel radius")
        if wheel["side"] not in ("left", "right"):
            raise ValueError("wheel side")
        carrier_side = (wheel["carrier_body_name"], wheel["side"])
        if carrier_side in carrier_sides:
            raise ValueError("duplicate carrier side")
        carrier_sides.add(carrier_side)
        joint = wheel["spin_joint_name"]
        if joint is not None and (not isinstance(joint, str) or not joint):
            raise ValueError("spin joint")
        binding = wheel["spin_angle_scalar"]
        if binding is not None and (binding["name"] not in scalar_names or
                isinstance(binding["sign"], bool) or binding["sign"] not in (-1, 1) or
                scalars[scalar_names.index(binding["name"])]["unit"] != "rad"):
            raise ValueError("wheel spin scalar")
    bundle = document["resource_bundle"]
    if bundle is not None and (not isinstance(bundle, str) or not bundle):
        raise ValueError("resource bundle")
    return document


def decode_state(payload):
    count = struct.unpack_from("<I", payload)[0]
    offset = 4 + count * BODY.size
    scalar_count = struct.unpack_from("<I", payload, offset)[0]
    if len(payload) != offset + 4 + scalar_count * 9:
        raise ValueError("state size")
    bodies = [list(BODY.unpack_from(payload, 4 + i * BODY.size)) for i in range(count)]
    if any(not math.isfinite(value) for body in bodies for value in body):
        raise ValueError("nonfinite state")
    scalars = list(struct.unpack_from(f"<{scalar_count}d", payload, offset + 4))
    statuses = list(payload[offset + 4 + scalar_count * 8 :])
    if any(not math.isfinite(value) or status not in (0, 1, 2) or (status != 1 and value != 0)
           for value, status in zip(scalars, statuses, strict=True)):
        raise ValueError("scalar value/status")
    return {"bodies": bodies, "scalar_values": scalars, "scalar_statuses": statuses}


class RunReport:
    """Online statistics for complete messages; no state history is retained."""

    def __init__(self):
        self.descriptions = 0
        self.states = 0
        self.unbound_states = 0
        self.first_sequence = self.last_sequence = None
        self.first_time = self.last_time = self.minimum_time = self.maximum_time = None
        self.last_arrival = None
        self.gaps = self.same_time = self.backwards = 0
        self.interval_sum = self.interval_max = 0.0
        self.status_counts = []

    def observe(self, message, descriptor, arrival):
        if message["kind"] == "description":
            self.descriptions += 1
            if not self.status_counts:
                self.status_counts = [[0, 0, 0] for _ in descriptor["description"]["scalars"]]
            return
        sequence = int(message["sequence"])
        seconds = message["simulation_time_seconds"]
        if self.states:
            self.gaps += sequence - self.last_sequence - 1
            self.same_time += seconds == self.last_time
            self.backwards += seconds < self.last_time
            interval = arrival - self.last_arrival
            self.interval_sum += interval
            self.interval_max = max(self.interval_max, interval)
            self.minimum_time = min(self.minimum_time, seconds)
            self.maximum_time = max(self.maximum_time, seconds)
        else:
            self.first_sequence = sequence
            self.first_time = self.minimum_time = self.maximum_time = seconds
        self.states += 1
        self.last_sequence, self.last_time, self.last_arrival = sequence, seconds, arrival
        if descriptor is None:
            self.unbound_states += 1
        else:
            for counts, status in zip(self.status_counts, message["scalar_statuses"], strict=True):
                counts[status] += 1

    def snapshot(self, run_id, descriptor):
        common = descriptor["description"] if descriptor else None
        scalars = common["scalars"] if common else []
        indices = {field["name"]: i for i, field in enumerate(scalars)}

        def field(name):
            index = indices.get(name)
            return {"name": name, "provided": index is not None, "index": index,
                    "unit": scalars[index]["unit"] if index is not None else None,
                    "status_counts": dict(zip(("not_ready", "valid", "placeholder"), self.status_counts[index]))
                    if index is not None else None}

        vocabulary = []
        wheels = []
        if common:
            vocabulary = [field(body["name"] + "." + suffix) for body in common["bodies"]
                          for suffix in ("track_station_meters", "lateral_meters", "yaw_radians")]
            vocabulary += [field(wheel["interface_name"] + "." + suffix) for wheel in common["wheel_placements"]
                           for suffix in ("contact_patch_count", "vertical_support_force_on_wheel_newtons", "normal_force_newtons")]
            for wheel in common["wheel_placements"]:
                binding = wheel["spin_angle_scalar"]
                wheels.append({"interface_name": wheel["interface_name"],
                               "binding": None if binding is None else {**field(binding["name"]), "sign": binding["sign"]}})
        return {
            "run_id": str(run_id), "description_messages": self.descriptions,
            "body_count": len(common["bodies"]) if common else None,
            "wheel_count": len(common["wheel_placements"]) if common else None,
            "scalar_count": len(scalars) if common else None,
            "resource_bundle": descriptor["resource_bundle"] if descriptor else None,
            "nominal_sample_period_seconds": common["nominal_sample_period_seconds"] if common else None,
            "fields": [field(scalar["name"]) for scalar in scalars],
            "vocabulary": vocabulary, "wheel_spin_bindings": wheels,
            "states": self.states, "states_without_description": self.unbound_states,
            "first_sequence": str(self.first_sequence) if self.states else None,
            "last_sequence": str(self.last_sequence) if self.states else None,
            "sequence_gaps": self.gaps, "same_time_states": self.same_time,
            "backwards_time_states": self.backwards,
            "simulation_time_seconds": {"first": self.first_time, "last": self.last_time,
                                        "minimum": self.minimum_time, "maximum": self.maximum_time},
            "arrival_interval_seconds": {
                "mean": self.interval_sum / (self.states - 1) if self.states > 1 else None,
                "maximum": self.interval_max if self.states > 1 else None},
        }


class Receiver:
    """Bounded reassembly, ordered independently for each run and message kind."""

    def __init__(self, *, maximum_message_bytes=1024 * 1024,
                 maximum_pending_messages=8, maximum_runs=16, report=False):
        if any(not isinstance(value, int) or isinstance(value, bool) or value < 1 for value in
               (maximum_message_bytes, maximum_pending_messages, maximum_runs)):
            raise ValueError("receiver limits must be positive integers")
        self.maximum_message_bytes = maximum_message_bytes
        self.maximum_pending_messages = maximum_pending_messages
        self.maximum_runs = maximum_runs
        self.pending = OrderedDict()
        self.runs = OrderedDict()
        self.latest = {}
        self.descriptions = {}
        self.reports = {} if report else None
        self.evicted_runs = 0
        self.statistics = {
            "datagrams": 0, "invalid_datagrams": 0, "duplicate_or_old_datagrams": 0,
            "incomplete_messages_evicted": 0, "state_frames": 0, "sequence_gaps": 0,
        }

    def feed(self, packet, *, received_at=None):
        self.statistics["datagrams"] += 1
        try:
            message = self._feed(packet)
        except (ValueError, struct.error, UnicodeError, KeyError, TypeError, OverflowError):
            self.statistics["invalid_datagrams"] += 1
            return None
        if message is not None and self.reports is not None:
            run_id = int(message["run_id"])
            report = self.reports.get(run_id)
            if report is None:
                report = self.reports[run_id] = RunReport()
            report.observe(message, self.descriptions.get(run_id),
                           time.monotonic() if received_at is None else received_at)
        return message

    def report(self):
        return {"statistics": dict(self.statistics), "retained_runs": len(self.runs),
                "evicted_runs": self.evicted_runs,
                "runs": [(self.reports.get(run) or RunReport()).snapshot(run, self.descriptions.get(run))
                         for run in self.runs] if self.reports is not None else []}

    def _touch_run(self, run_id):
        self.runs[run_id] = None
        self.runs.move_to_end(run_id)
        if len(self.runs) > self.maximum_runs:
            oldest, _ = self.runs.popitem(last=False)
            self.evicted_runs += 1
            if self.reports is not None:
                self.reports.pop(oldest, None)
            self.descriptions.pop(oldest, None)
            self.latest = {key: value for key, value in self.latest.items() if key[0] != oldest}
            for key in list(self.pending):
                if key[0] == oldest:
                    del self.pending[key]
                    self.statistics["incomplete_messages_evicted"] += 1

    def _feed(self, packet):
        if not HEADER.size <= len(packet) <= 1200:
            raise ValueError("packet length")
        magic, version, kind, run_id, seq, sim_time, part, parts, total = HEADER.unpack_from(packet)
        if magic != b"OSTS" or version != 1 or kind not in (1, 2) or run_id == 0 or not math.isfinite(sim_time):
            raise ValueError("header")
        if total > self.maximum_message_bytes:
            raise ValueError("message byte limit")
        if parts != max(1, (total + CHUNK - 1) // CHUNK) or part >= parts:
            raise ValueError("parts")
        payload = packet[HEADER.size :]
        if len(payload) != min(CHUNK, total - part * CHUNK):
            raise ValueError("part length")
        self._touch_run(run_id)
        lane = (run_id, kind)
        if seq <= self.latest.get(lane, -1):
            self.statistics["duplicate_or_old_datagrams"] += 1
            return None
        key = (run_id, kind, seq)
        if key not in self.pending:
            self.pending[key] = (sim_time, parts, total, {})
            if len(self.pending) > self.maximum_pending_messages:
                self.pending.popitem(last=False)
                self.statistics["incomplete_messages_evicted"] += 1
        prior_time, prior_parts, prior_total, chunks = self.pending[key]
        if (sim_time, parts, total) != (prior_time, prior_parts, prior_total):
            del self.pending[key]
            raise ValueError("conflicting fragments")
        if part in chunks:
            if chunks[part] != payload:
                del self.pending[key]
                raise ValueError("conflicting duplicate")
            self.statistics["duplicate_or_old_datagrams"] += 1
            return None
        chunks[part] = payload
        if len(chunks) != parts:
            return None
        del self.pending[key]
        payload = b"".join(chunks[index] for index in range(parts))
        result = {"run_id": str(run_id), "sequence": str(seq), "simulation_time_seconds": sim_time}
        if kind == 1:
            descriptor = decode_description(payload)
            previous = self.descriptions.get(run_id)
            if previous is not None and previous != descriptor:
                raise ValueError("description changed within a run")
            self.descriptions[run_id] = descriptor
            result.update(kind="description", description=descriptor)
        else:
            state = decode_state(payload)
            descriptor = self.descriptions.get(run_id)
            if descriptor is not None:
                common = descriptor["description"]
                if len(common["bodies"]) != len(state["bodies"]) or len(common["scalars"]) != len(state["scalar_values"]):
                    raise ValueError("description count")
                state["body_names"] = [body["name"] for body in common["bodies"]]
                state["scalar_names"] = [field["name"] for field in common["scalars"]]
            previous = self.latest.get(lane)
            if previous is not None:
                self.statistics["sequence_gaps"] += seq - previous - 1
            self.statistics["state_frames"] += 1
            result.update(kind="state", **state)
        self.latest[lane] = seq
        for old in list(self.pending):
            if old[:2] == lane and old[2] <= seq:
                del self.pending[old]
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=10099)
    parser.add_argument("--output-directory", type=Path)
    parser.add_argument("--report", type=Path, help="Write an online field and cadence report; refuse overwrite")
    parser.add_argument("--duration-seconds", type=float, help="Stop this many seconds after binding")
    parser.add_argument("--max-states", type=int, help="Stop after this many complete decoded states")
    parser.add_argument("--capture-states", action="store_true")
    parser.add_argument(
        "--idle-seconds",
        type=float,
        default=2.0,
        help="Finish after this quiet interval following the first datagram",
    )
    args = parser.parse_args()
    if args.output_directory is None and args.report is None:
        parser.error("--output-directory or --report is required")
    if args.capture_states and args.output_directory is None:
        parser.error("--capture-states requires --output-directory")
    for name in ("idle_seconds", "duration_seconds"):
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or value <= 0):
            parser.error(f"--{name.replace('_', '-')} must be positive and finite")
    if args.max_states is not None and args.max_states < 1:
        parser.error("--max-states must be positive")
    if args.report is not None and args.report.exists():
        parser.error("--report already exists")
    receiver = Receiver(report=args.report is not None)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as channel:
        channel.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        channel.bind((args.bind, args.port))
        started = time.monotonic()
        if args.output_directory is not None:
            args.output_directory.mkdir(parents=True, exist_ok=False)
        capture = ((args.output_directory / "states.jsonl").open("x") if args.capture_states else None)
        first = last = last_packet = None
        reason = "interrupted"
        try:
            print(json.dumps({"listening": channel.getsockname()}), flush=True)
            while True:
                now = time.monotonic()
                remaining = args.duration_seconds - (now - started) if args.duration_seconds is not None else math.inf
                idle = args.idle_seconds - (now - last_packet) if last_packet is not None else math.inf
                if remaining <= 0:
                    reason = "duration"
                    break
                if idle <= 0:
                    reason = "idle"
                    break
                channel.settimeout(min(0.25, remaining, idle))
                try:
                    packet, _source = channel.recvfrom(65535)
                except TimeoutError:
                    continue
                last_packet = time.monotonic()
                decoded = receiver.feed(packet, received_at=last_packet)
                if decoded is None:
                    continue
                if decoded["kind"] == "description":
                    if args.output_directory is not None:
                        destination = args.output_directory / f"description-{decoded['run_id']}.json"
                        destination.write_text(json.dumps(decoded, indent=2) + "\n")
                else:
                    if args.output_directory is not None:
                        first = decoded if first is None else first
                        last = decoded
                    if capture:
                        capture.write(json.dumps(decoded, allow_nan=False) + "\n")
                    if args.max_states is not None and receiver.statistics["state_frames"] >= args.max_states:
                        reason = "max_states"
                        break
        except KeyboardInterrupt:
            pass
        finally:
            if capture:
                capture.close()
        if args.output_directory is not None:
            summary = {**receiver.statistics, "incomplete_messages_pending": len(receiver.pending),
                       "first_state": first, "last_state": last}
            (args.output_directory / "receiver_summary.json").write_text(
                json.dumps(summary, indent=2, allow_nan=False) + "\n")
        if args.report is not None:
            with args.report.open("x") as report_file:
                json.dump({**receiver.report(), "exit_reason": reason,
                           "listening_seconds": time.monotonic() - started}, report_file,
                          indent=2, allow_nan=False)
                report_file.write("\n")
        print(json.dumps(receiver.statistics), flush=True)


if __name__ == "__main__":
    main()
