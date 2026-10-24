import { contractFixture } from './scene_contract_fixtures.ts';
import { fragmentPayloadBytes, headerBytes } from '../src/stream/protocol.ts';
export function datagrams(kind: 1 | 2, payload: Uint8Array, runId = 1n, sequence = 1n, time = 0): Uint8Array[] {
  const count = Math.max(1, Math.ceil(payload.length / fragmentPayloadBytes));
  return Array.from({ length: count }, (_, part) => {
    const data = payload.subarray(part * fragmentPayloadBytes, (part + 1) * fragmentPayloadBytes);
    const bytes = new Uint8Array(headerBytes + data.length); const view = new DataView(bytes.buffer);
    view.setUint32(0, 0x5354534f, true); view.setUint16(4, 1, true); view.setUint16(6, kind, true);
    view.setBigUint64(8, runId, true); view.setBigUint64(16, sequence, true); view.setFloat64(24, time, true);
    view.setUint16(32, part, true); view.setUint16(34, count, true); view.setUint32(36, payload.length, true);
    bytes.set(data, headerBytes); return bytes;
  });
}
export function complete(packets: Uint8Array[]): Uint8Array {
  const length = packets.reduce((sum, packet) => sum + packet.length - headerBytes, headerBytes);
  const result = new Uint8Array(length); result.set(packets[0]!.subarray(0, headerBytes)); let offset = headerBytes;
  for (const packet of packets) { result.set(packet.subarray(headerBytes), offset); offset += packet.length - headerBytes; }
  return result;
}
export function descriptionPayload(resourceBundle: string | null = null): Uint8Array {
  return new TextEncoder().encode(JSON.stringify({ schema: 'orvd.scene_description', description: contractFixture().json.description, resource_bundle: resourceBundle }) + ' '.repeat(800));
}
export function statePayload(bodyCount = 1, scalarValues = [2], statuses = [1]): Uint8Array {
  const body = new Uint8Array(8 + bodyCount * 104 + scalarValues.length * 9); const view = new DataView(body.buffer);
  view.setUint32(0, bodyCount, true);
  for (let i = 0; i < bodyCount; ++i) view.setFloat64(4 + i * 104 + 24, 1, true);
  const start = 4 + bodyCount * 104; view.setUint32(start, scalarValues.length, true);
  for (let i = 0; i < scalarValues.length; ++i) {
    view.setFloat64(start + 4 + i * 8, scalarValues[i]!, true); view.setUint8(start + 4 + scalarValues.length * 8 + i, statuses[i]!);
  }
  return body;
}
