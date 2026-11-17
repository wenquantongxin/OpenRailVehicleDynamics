import { rowPathOfSubject } from '../service/find_model_tree_rows.ts';
import { elementKey, subjectElement, type ElementReference } from '../service/vehicle_modeling_service_protocol.ts';
import type { EditRefusal, SessionState } from '../session/vehicle_editing_session.ts';
import type { VisualPart } from '../visual/read_vehicle_visual_definition.ts';
import { BilingualTitleText } from './BilingualTitleText.tsx';
import { CollapsibleMessageGroup } from './CollapsibleMessageGroup.tsx';
import { DockedPanelTitleStrip } from './DockedPanelTitleStrip.tsx';
import { ElementLinkButton } from './ElementLinkButton.tsx';
import { elementKindTitle } from './find_element_kind_title.ts';
import { identifierLabel } from './identifier_display_labels.ts';
import { openChoiceInSentence } from './open_choice_and_edited_project_wording.ts';
import { WrappableElementName } from './WrappableElementName.tsx';

// What the service said: the latest check (refusals first, then information,
// then what the check covers), the latest refused edit and command, what the
// latest removal took, and what the preview could not draw; and the visual
// parts left without a body to hang on. Check refusals are the kernel's
// sentences and are shown as they are. Besides, two notices of the session's
// own: a link the page was opened with that was not followed because a
// session was open, and a visual definition the view cannot draw from.

export function ServiceMessagesPanel(props: {
  state: SessionState;
  /** Visual definition parts whose body the drawing does not have. */
  partsWithoutBody: VisualPart[];
  onSelect: (element: ElementReference) => void;
  onShowRefusal: (refusal: EditRefusal) => void;
}) {
  const { state } = props;
  const report = state.check?.report ?? null;
  const notDrawable = state.preview?.notDrawableElements ?? [];
  const refusedElement = subjectElement(state.editRefusal?.subject ?? null);
  const refusedSettingsTitle = state.editRefusal === null || refusedElement !== null ? null : (rowPathOfSubject(state.modelTree ?? [], state.editRefusal.subject)?.at(-1)?.title ?? null);
  return (
    <section className="panel messages-panel" aria-label="Messages">
      <DockedPanelTitleStrip en="Messages" zh="消息" detail={state.check === null ? undefined : `check at revision ${state.check.revision}`} />
      <div className="messages">
        {state.failure !== null && (
          <div className="message failure" role="alert">
            <b>
              Session stopped <span className="zh">会话已停止</span>
            </b>
            <span>{state.failure}</span>
            <span className="hint">
              Restart the modelling server and reload the page. <span className="zh">请重启建模服务器并刷新页面。</span>
            </span>
          </div>
        )}
        {state.commandRefusal !== null && (
          <div className="message refused" role="alert">
            <b>
              Refused <span className="zh">被拒绝</span>
            </b>
            <span>{state.commandRefusal.message}</span>
          </div>
        )}
        {state.linkNotFollowed !== null && (
          <div className="message notice" role="status">
            <b>
              Link not followed <span className="zh">链接未执行</span>
            </b>
            <span>
              The page was opened with a link to {openChoiceInSentence(state.linkNotFollowed).en}, but the service had a session open; it has been
              resumed as it was.
            </span>
            <span className="zh">页面链接要求打开{openChoiceInSentence(state.linkNotFollowed).zh}，但服务中已有打开的会话，已按原样恢复。</span>
          </div>
        )}
        {state.vehicle !== null && state.vehicle.visualDefinitionProblem !== null && (
          <div className="message notice" role="status">
            <b>
              Visual definition not drawn <span className="zh">外观定义无法绘制</span>
            </b>
            <span>{state.vehicle.visualDefinitionProblem}</span>
            <span className="hint">
              The bodies are drawn as their markers; editing goes on as usual. <span className="zh">刚体以标记绘制，编辑照常进行。</span>
            </span>
          </div>
        )}
        {state.removedElements !== null && (
          <section className="message-group">
            <h3 className="group-title">
              Removed <span className="zh">已删除</span> <span className="count">{state.removedElements.length}</span>
            </h3>
            <p className="removal-summary">
              {state.removedElements.map((element) => (
                <span key={elementKey(element)} className="removed-element">
                  <span className="link-kind">{elementKindTitle(state.elementKinds, element.kind).en}</span> <WrappableElementName name={element.name} />
                </span>
              ))}
            </p>
            <p className="removal-note">
              Undo brings them back. <span className="zh">撤销可恢复。</span>
            </p>
          </section>
        )}
        {state.editRefusal !== null && (
          <button type="button" className="message refused edit-refusal" onClick={() => props.onShowRefusal(state.editRefusal!)}>
            <b>
              Edit refused <span className="zh">修改被拒绝</span>
            </b>
            <span className="refused-target">
              {refusedElement !== null && <WrappableElementName name={refusedElement.name} />}
              {refusedSettingsTitle !== null && <BilingualTitleText title={refusedSettingsTitle} />} · {identifierLabel(state.editRefusal.property)}
            </span>
            <span>{state.editRefusal.message}</span>
          </button>
        )}
        {report !== null && (
          <>
            {report.refusals.length > 0 && (
              <section className="message-group">
                <h3 className="group-title failed">
                  Check refusals <span className="zh">检查未通过的原因</span> <span className="count">{report.refusals.length}</span>
                </h3>
                <ul className="check-refusals">
                  {report.refusals.map((refusal, index) => (
                    <li key={index}>{refusal}</li>
                  ))}
                </ul>
              </section>
            )}
            {report.information.length > 0 && (
              <section className="message-group">
                <h3 className="group-title">
                  Information <span className="zh">信息</span>
                </h3>
                <ul className="check-information">
                  {report.information.map((line, index) => (
                    <li key={index}>{line}</li>
                  ))}
                </ul>
              </section>
            )}
            <CollapsibleMessageGroup en="What the check covers" zh="检查范围" count={report.coverageStatements.length}>
              <ul className="coverage">
                {report.coverageStatements.map((statement, index) => (
                  <li key={index}>{statement}</li>
                ))}
              </ul>
            </CollapsibleMessageGroup>
          </>
        )}
        {notDrawable.length > 0 && (
          <section className="message-group">
            <h3 className="group-title">
              Not drawable <span className="zh">无法绘制</span> <span className="count">{notDrawable.length}</span>
            </h3>
            <div className="element-links">
              {notDrawable.map((element) => (
                <ElementLinkButton key={elementKey(element)} element={element} kindTitle={elementKindTitle(state.elementKinds, element.kind)} onSelect={props.onSelect} />
              ))}
            </div>
          </section>
        )}
        {props.partsWithoutBody.length > 0 && (
          <CollapsibleMessageGroup en="Visual parts without a drawn body" zh="外形部件无对应刚体" count={props.partsWithoutBody.length}>
            <ul className="parts-without-body">
              {props.partsWithoutBody.map((part) => (
                <li key={part.name}>
                  <WrappableElementName name={part.name} /> <span className="part-body">on {part.bodyName}</span>
                </li>
              ))}
            </ul>
          </CollapsibleMessageGroup>
        )}
        {state.vehicle === null && state.failure === null && state.commandRefusal === null && (
          <p className="panel-empty">
            Check results appear here once a vehicle is open. <span className="zh">打开车辆后在此显示检查结果。</span>
          </p>
        )}
      </div>
    </section>
  );
}
