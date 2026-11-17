import type { BilingualTitle } from '../service/vehicle_modeling_service_protocol.ts';

/** A title the service gives, English with its Chinese subtitle. */
export function BilingualTitleText({ title }: { title: BilingualTitle }) {
  return (
    <>
      <span className="en">{title.en}</span> <span className="zh">{title.zh}</span>
    </>
  );
}
