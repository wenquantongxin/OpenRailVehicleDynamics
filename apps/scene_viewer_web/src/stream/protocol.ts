import { parseSceneDescription } from '../record/scene_definition.ts';
import type { SceneDescription } from '../record/scene_definition.ts';

export const headerBytes = 40;
export const fragmentPayloadBytes = 1160;
export const maximumMessageBytes = 1024 * 1024;
export interface OstsHeader {
  kind: 1 | 2;
  runId: bigint;
  sequence: bigint;
  simulationTimeSeconds: number;
  fragmentIndex: number;
  fragmentCount: number;
  payloadBytes: number;
}
export interface StreamState {
  bodyCount: number;
  scalarCount: number;
  bodyValues: Float64Array;
  scalarValues: Float64Array;
  scalarStatuses: Uint8Array;
}
export interface DescriptionMessage {
  kind: 'description';
  header: OstsHeader;
  description: SceneDescription;
  resourceBundle: string | null;
  bytes: Uint8Array;
}
export interface StateMessage { kind: 'state'; header: OstsHeader; state: StreamState; bytes: Uint8Array; }
export type DecodedMessage = DescriptionMessage | StateMessage;

function parseHeader(bytes: Uint8Array): OstsHeader {
  if (bytes.byteLength < headerBytes) throw new Error('Truncated OSTS header');
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  if (view.getUint32(0, true) !== 0x5354534f || view.getUint16(4, true) !== 1) throw new Error('Invalid OSTS magic or version');
  const kind = view.getUint16(6, true);
  const runId = view.getBigUint64(8, true);
  const simulationTimeSeconds = view.getFloat64(24, true);
  if ((kind !== 1 && kind !== 2) || runId === 0n || !Number.isFinite(simulationTimeSeconds)) throw new Error('Invalid OSTS kind, run identity or time');
  const header: OstsHeader = { kind, runId, sequence: view.getBigUint64(16, true), simulationTimeSeconds,
    fragmentIndex: view.getUint16(32, true), fragmentCount: view.getUint16(34, true), payloadBytes: view.getUint32(36, true) };
  if (header.payloadBytes > maximumMessageBytes) throw new Error('OSTS message exceeds receiver limit');
  if (header.fragmentCount !== Math.max(1, Math.ceil(header.payloadBytes / fragmentPayloadBytes)) || header.fragmentIndex >= header.fragmentCount) {
    throw new Error('Invalid OSTS fragment indices');
  }
  return header;
}
export function decodeDatagramHeader(bytes: Uint8Array): OstsHeader {
  const header = parseHeader(bytes);
  const size = Math.min(fragmentPayloadBytes, header.payloadBytes - header.fragmentIndex * fragmentPayloadBytes);
  if (bytes.byteLength !== headerBytes + size) throw new Error('Invalid OSTS datagram length');
  return header;
}
export function decodeState(bytes: Uint8Array): StreamState {
  if (bytes.byteLength < 8) throw new Error('Truncated OSTS state');
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const bodyCount = view.getUint32(0, true);
  if (bodyCount > (bytes.byteLength - 8) / 104) throw new Error('OSTS body count exceeds payload');
  const scalarOffset = 4 + bodyCount * 104;
  const scalarCount = view.getUint32(scalarOffset, true);
  if (bytes.byteLength !== scalarOffset + 4 + scalarCount * 9) throw new Error('OSTS scalar count differs from payload');
  const bodyValues = new Float64Array(bodyCount * 13);
  const scalarValues = new Float64Array(scalarCount);
  const scalarStatuses = bytes.subarray(scalarOffset + 4 + scalarCount * 8);
  for (let i = 0; i < bodyValues.length; ++i) {
    const value = view.getFloat64(4 + i * 8, true);
    if (!Number.isFinite(value)) throw new Error('Nonfinite OSTS body value');
    bodyValues[i] = value;
  }
  for (let i = 0; i < scalarCount; ++i) {
    const value = view.getFloat64(scalarOffset + 4 + i * 8, true);
    const status = scalarStatuses[i]!;
    if (!Number.isFinite(value) || status > 2 || (status !== 1 && value !== 0)) throw new Error('Inconsistent OSTS scalar status/value');
    scalarValues[i] = value;
  }
  return { bodyCount, scalarCount, bodyValues, scalarValues, scalarStatuses };
}
/** Reliable transport envelope: original fragment-zero header and the complete body. */
export function decodeMessage(bytes: Uint8Array): DecodedMessage {
  const header = parseHeader(bytes);
  if (header.fragmentIndex !== 0 || bytes.byteLength !== headerBytes + header.payloadBytes) throw new Error('Invalid complete OSTS message length');
  const payload = bytes.subarray(headerBytes);
  if (header.kind === 1) {
    const parsed = parseSceneDescription(JSON.parse(new TextDecoder('utf-8', { fatal: true }).decode(payload)));
    return { kind: 'description', header, bytes, ...parsed };
  }
  return { kind: 'state', header, bytes, state: decodeState(payload) };
}
export function stateMatchesDescription(state: StreamState, description: SceneDescription): boolean {
  return state.bodyCount === description.bodies.length && state.scalarCount === description.scalars.length;
}
export function descriptionSemanticallyEqual(a: DescriptionMessage, b: DescriptionMessage): boolean {
  return a.resourceBundle === b.resourceBundle && JSON.stringify(a.description) === JSON.stringify(b.description);
}
