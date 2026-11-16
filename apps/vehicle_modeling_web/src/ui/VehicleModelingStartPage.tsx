import type { ReferenceVehicle } from '../service/fetch_reference_vehicles.ts';
import type { OpenChoice } from '../session/vehicle_editing_session.ts';
import { DockedPanelTitleStrip } from './DockedPanelTitleStrip.tsx';

// What can be opened, as two different things side by side: the reference
// vehicles to start from, and one's own projects saved in the folder of
// projects. Shown when nothing is open, and over the open session when Open
// is asked for; then it can be left back to the session without opening
// anything.

export function VehicleModelingStartPage(props: {
  vehicles: ReferenceVehicle[] | null;
  /** The projects of the folder, sorted by the service; null until listed. */
  vehicleProjects: string[] | null;
  vehicleProjectsDirectory: string | null;
  /** Why something cannot be offered, or why the last open was refused. */
  problem: string | null;
  /** The name of what the session edits, when the page is shown over it; null when nothing is open. */
  shownOver: string | null;
  onOpen: (choice: OpenChoice) => void;
  onLeave: () => void;
}) {
  const back = props.shownOver === null ? undefined : (
    <button type="button" className="chip back-button" onClick={props.onLeave}>
      <span className="en">Back to {props.shownOver}</span>
      <span className="zh">返回</span>
    </button>
  );
  return (
    <div className={props.shownOver === null ? 'open-stage' : 'open-stage over-session'}>
      <section className="open-card" aria-label="Open">
        <DockedPanelTitleStrip en="Open" zh="打开" detail={back} />
        {props.problem !== null && <p className="open-problem">{props.problem}</p>}
        <div className="open-columns">
          <section className="open-column">
            <h2 className="open-column-title">
              Start from a reference vehicle <span className="zh">从参考车辆开始</span>
            </h2>
            <p className="open-column-note">
              Its files are only read: save what you make of it as a project of your own.
              <span className="zh">参考车辆的文件只读不写；在其基础上的修改另存为自己的工程。</span>
            </p>
            <div className="open-choices">
              {props.vehicles !== null && props.vehicles.length === 0 && (
                <p className="open-empty">
                  The server was started without reference vehicles. <span className="zh">服务器启动时未给出参考车辆。</span>
                </p>
              )}
              {(props.vehicles ?? []).map((vehicle) => (
                <button type="button" key={vehicle.name} className="open-choice" onClick={() => props.onOpen({ source: 'reference_vehicle', vehicle })}>
                  <span className="en">{vehicle.title.en}</span>
                  <span className="zh">{vehicle.title.zh}</span>
                  <span className="choice-tag">{vehicle.name}</span>
                </button>
              ))}
            </div>
          </section>
          <section className="open-column vehicle-projects">
            <h2 className="open-column-title">
              Your projects <span className="zh">我的工程</span>
            </h2>
            <p className="open-column-note">
              Saved in <span className="projects-folder">{props.vehicleProjectsDirectory ?? '—'}</span>
              <span className="zh">保存在上述工程文件夹中。</span>
            </p>
            {props.vehicleProjects !== null && props.vehicleProjects.length === 0 && (
              <p className="open-empty">
                No project has been saved in the folder yet. <span className="zh">文件夹中尚无已保存的工程。</span>
              </p>
            )}
            {props.vehicleProjects !== null && props.vehicleProjects.length > 0 && (
              <ul className="project-choices">
                {props.vehicleProjects.map((name) => (
                  <li key={name}>
                    <button type="button" className="project-choice" onClick={() => props.onOpen({ source: 'vehicle_project', name })}>
                      {name}
                    </button>
                  </li>
                ))}
              </ul>
            )}
          </section>
        </div>
      </section>
    </div>
  );
}
