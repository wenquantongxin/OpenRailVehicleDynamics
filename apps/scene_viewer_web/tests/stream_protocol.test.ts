import assert from 'node:assert/strict';
import { test } from 'node:test';
import { decodeDatagramHeader, decodeMessage, headerBytes, maximumMessageBytes } from '../src/stream/protocol.ts';
import { StreamReceiver } from '../src/stream/receiver.ts';
import { datagrams, complete, descriptionPayload, statePayload } from './stream_fixtures.ts';
const feed = (receiver: StreamReceiver, packets: Uint8Array[]) => packets.reduce<ReturnType<StreamReceiver['feed']>>((last, packet) => receiver.feed(packet) ?? last, null);

test('UDP and reliable envelopes validate their own lengths and enforce limits before buffering', () => {
  const packet = datagrams(1, descriptionPayload())[0]!;
  assert.doesNotThrow(() => decodeDatagramHeader(packet)); assert.throws(() => decodeMessage(packet), /complete/);
  assert.throws(() => decodeDatagramHeader(complete(datagrams(1, descriptionPayload()))), /datagram/);
  const tooLarge = packet.slice(); new DataView(tooLarge.buffer).setUint32(36, maximumMessageBytes + 1, true);
  const receiver = new StreamReceiver(); assert.throws(() => receiver.feed(tooLarge), /limit/);
  assert.equal(receiver.partialCount, 0); assert.equal(receiver.runCount, 0);
  const bad = packet.slice(); bad[0] = 0; assert.throws(() => receiver.feed(bad), /magic/);
});

test('invalid high sequence and incompatible counts never advance the accepted lane', () => {
  const receiver = new StreamReceiver(); feed(receiver, datagrams(1, descriptionPayload()));
  const invalid = statePayload(); new DataView(invalid.buffer).setFloat64(4, NaN, true);
  assert.throws(() => feed(receiver, datagrams(2, invalid, 1n, 999n)), /Nonfinite/);
  assert.throws(() => feed(receiver, datagrams(2, statePayload(2), 1n, 998n)), /counts/);
  assert.equal(feed(receiver, datagrams(2, statePayload(), 1n, 2n))?.state.header.sequence, 2n);
  assert.equal(feed(receiver, datagrams(2, statePayload(), 1n, 2n)), null);
  for (const invalidState of [statePayload(1, [3], [0]), statePayload(1, [0], [3])]) assert.throws(() => feed(receiver, datagrams(2, invalidState, 1n, 997n)), /status/);
});

test('description-late states require matching counts without poisoning the lane', () => {
  const receiver = new StreamReceiver(); assert.equal(feed(receiver, datagrams(2, statePayload(2), 1n, 100n)), null);
  assert.equal(feed(receiver, datagrams(1, descriptionPayload())), null);
  assert.equal(feed(receiver, datagrams(2, statePayload(), 1n, 1n))?.changedRun, true);
  const late = new StreamReceiver(); assert.equal(feed(late, datagrams(2, statePayload(), 9n, 10n)), null);
  assert.equal(feed(late, datagrams(1, descriptionPayload(), 9n))?.state.header.sequence, 10n);
});

test('new ready runs replace the display; retired runs and description repeats cannot switch back', () => {
  const receiver = new StreamReceiver(); feed(receiver, datagrams(1, descriptionPayload())); feed(receiver, datagrams(2, statePayload()));
  feed(receiver, datagrams(1, descriptionPayload(), 2n)); assert.equal(receiver.current?.state.header.runId, 1n);
  assert.equal(feed(receiver, datagrams(2, statePayload(), 2n))?.changedRun, true);
  assert.equal(feed(receiver, datagrams(1, descriptionPayload(), 1n, 100n)), null);
  assert.equal(feed(receiver, datagrams(2, statePayload(), 1n, 100n)), null); assert.equal(receiver.current?.state.header.runId, 2n);
  assert.throws(() => feed(receiver, datagrams(1, descriptionPayload('different'), 2n, 100n)), /changed/);
  assert.equal(feed(receiver, datagrams(1, descriptionPayload(), 2n, 2n)), null);
  assert.equal(feed(receiver, datagrams(2, statePayload(), 2n, 2n))?.changedRun, false);
});

for (const firstKind of [1, 2] as const) {
  test(`an earlier ${firstKind === 1 ? 'description' : 'state'}-only run cannot take over after its delayed counterpart arrives`, () => {
    const receiver = new StreamReceiver();
    const earlierId = 2n ** 63n + 7n;
    const laterId = 3n;
    const firstPayload = firstKind === 1 ? descriptionPayload() : statePayload();
    assert.equal(feed(receiver, datagrams(firstKind, firstPayload, earlierId)), null);
    feed(receiver, datagrams(1, descriptionPayload(), laterId));
    assert.equal(feed(receiver, datagrams(2, statePayload(), laterId))?.changedRun, true);
    const counterpartKind = firstKind === 1 ? 2 : 1;
    const counterpart = counterpartKind === 1 ? descriptionPayload() : statePayload();
    assert.equal(feed(receiver, datagrams(counterpartKind, counterpart, earlierId)), null);
    for (const sequence of [2n, 1000n]) {
      assert.equal(feed(receiver, datagrams(1, descriptionPayload(), earlierId, sequence)), null);
      assert.equal(feed(receiver, datagrams(2, statePayload(), earlierId, sequence)), null);
    }
    assert.equal(receiver.current?.state.header.runId, laterId);
  });
}

test('activation retires only earlier runs and leaves newer pending runs eligible', () => {
  const receiver = new StreamReceiver();
  feed(receiver, datagrams(1, descriptionPayload(), 20n));
  feed(receiver, datagrams(2, statePayload(), 2n));
  assert.equal(feed(receiver, datagrams(2, statePayload(), 20n))?.state.header.runId, 20n);
  assert.equal(feed(receiver, datagrams(1, descriptionPayload(), 2n))?.state.header.runId, 2n);
  assert.equal(feed(receiver, datagrams(2, statePayload(), 20n, 2n)), null);
});

test('observation order starts at a complete valid message, not a fragment or malformed payload', () => {
  const receiver = new StreamReceiver();
  const incomplete = datagrams(1, descriptionPayload(), 70n);
  receiver.feed(incomplete[0]!);
  assert.throws(() => feed(receiver, datagrams(1, new TextEncoder().encode('invalid'), 80n)));
  feed(receiver, datagrams(1, descriptionPayload(), 90n));
  feed(receiver, datagrams(2, statePayload(), 90n));
  assert.equal(receiver.runCount, 1);
  for (const packet of incomplete.slice(1)) receiver.feed(packet);
  assert.equal(feed(receiver, datagrams(2, statePayload(), 70n))?.state.header.runId, 70n);
  feed(receiver, datagrams(1, descriptionPayload(), 80n));
  assert.equal(feed(receiver, datagrams(2, statePayload(), 80n))?.state.header.runId, 80n);
});

test('run eviction keeps the active display and the remaining pending-run order bounded', () => {
  const receiver = new StreamReceiver();
  feed(receiver, datagrams(1, descriptionPayload(), 999n));
  feed(receiver, datagrams(2, statePayload(), 999n));
  for (let id = 1n; id <= 30n; ++id) feed(receiver, datagrams(1, descriptionPayload(), id));
  assert.equal(receiver.runCount, 16);
  assert.equal(receiver.current?.state.header.runId, 999n);
  assert.equal(feed(receiver, datagrams(2, statePayload(), 30n))?.state.header.runId, 30n);
  assert.equal(feed(receiver, datagrams(2, statePayload(), 29n)), null);
  assert.equal(feed(receiver, datagrams(2, statePayload(), 999n, 2n)), null);
  assert.equal(receiver.current?.state.header.runId, 30n);
  assert.equal(receiver.runCount, 16);
  assert.ok(receiver.partialCount <= 8);
});

test('fragment conflicts, missing packets and receiver limits keep bounded state', () => {
  const receiver = new StreamReceiver(); const packets = datagrams(1, descriptionPayload()); receiver.feed(packets[0]!);
  const conflict = packets[0]!.slice(); conflict[headerBytes] = conflict[headerBytes]! ^ 1;
  assert.throws(() => receiver.feed(conflict), /Conflicting/); assert.equal(receiver.partialCount, 0);
  for (let i = 1; i <= 30; ++i) receiver.feed(datagrams(1, descriptionPayload(), BigInt(i))[0]!);
  assert.equal(receiver.runCount, 0); assert.equal(receiver.partialCount, 8);
  for (let i = 1; i <= 30; ++i) feed(receiver, datagrams(1, descriptionPayload(), BigInt(i)));
  assert.equal(receiver.runCount, 16); assert.ok(receiver.partialCount <= 8);
  const other = new StreamReceiver(); feed(other, datagrams(1, descriptionPayload()));
  other.feed(datagrams(2, statePayload(30), 1n, 1n)[0]!); assert.equal(other.partialCount, 1);
  feed(other, datagrams(2, statePayload(), 1n, 2n)); assert.equal(other.partialCount, 0);
});


test('incomplete and malformed new runs cannot evict retired-run history', () => {
  const receiver = new StreamReceiver();
  for (const id of [1n, 2n]) {
    feed(receiver, datagrams(1, descriptionPayload(), id));
    feed(receiver, datagrams(2, statePayload(), id));
  }
  for (let i = 3; i < 40; ++i) {
    receiver.feed(datagrams(1, descriptionPayload(), BigInt(i))[0]!);
    assert.throws(() => feed(receiver, datagrams(1, new TextEncoder().encode('invalid'), BigInt(i))));
  }
  assert.equal(receiver.runCount, 2);
  assert.equal(feed(receiver, datagrams(1, descriptionPayload(), 1n, 2n)), null);
  assert.equal(feed(receiver, datagrams(2, statePayload(), 1n, 2n)), null);
  assert.equal(receiver.current?.state.header.runId, 2n);
});
