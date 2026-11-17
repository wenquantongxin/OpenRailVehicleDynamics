import type { ElementReference } from '../service/vehicle_modeling_service_protocol.ts';
import { WrappableElementName } from './WrappableElementName.tsx';

/** What a change does to the joint the body hangs on: removes it when the body becomes free, or puts the new joint in its place. */
export function CurrentJointRemovalOrReplacementLine(props: { joint: ElementReference; removed: boolean }) {
  return (
    <p className="attachment-consequence">
      {props.removed ? (
        <>
          This change removes <WrappableElementName name={props.joint.name} />. <span className="zh">此更改将删除“{props.joint.name}”。</span>
        </>
      ) : (
        <>
          The new one takes the place of <WrappableElementName name={props.joint.name} />. <span className="zh">新的将取代“{props.joint.name}”。</span>
        </>
      )}
    </p>
  );
}
