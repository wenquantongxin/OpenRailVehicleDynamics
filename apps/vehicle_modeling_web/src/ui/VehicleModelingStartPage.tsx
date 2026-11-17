import type { ProjectStartingPoint } from '../service/fetch_project_starting_points.ts';
import type { OpenChoice } from '../session/vehicle_editing_session.ts';
import { BilingualTitleText } from './BilingualTitleText.tsx';
import { DockedPanelTitleStrip } from './DockedPanelTitleStrip.tsx';
import { startingPointsByCategory } from './group_project_starting_points_by_category.ts';

// What can be started or opened, as two different things side by side: the
// starting points a new project starts from, under their categories, and
// one's own projects saved in the folder of projects. Shown when nothing is
// open, and over the open session when New / Open is asked for; then it can
// be left back to the session without opening anything.

export function VehicleModelingStartPage(props: {
  startingPoints: ProjectStartingPoint[] | null;
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
      <section className="open-card" aria-label="New or open">
        <DockedPanelTitleStrip en="New / Open" zh="新建/打开" detail={back} />
        {props.problem !== null && <p className="open-problem">{props.problem}</p>}
        <div className="open-columns">
          <section className="open-column">
            <h2 className="open-column-title">
              New project <span className="zh">新建工程</span>
            </h2>
            <p className="open-column-note">
              A starting point is read and never written: what you build on it is saved as a project of your own.
              <span className="zh">起点只读不写；在其上建立的内容另存为自己的工程。</span>
            </p>
            {props.startingPoints !== null && props.startingPoints.length === 0 && (
              <p className="open-empty">
                The server was started without starting points. <span className="zh">服务器启动时未给出起点。</span>
              </p>
            )}
            {startingPointsByCategory(props.startingPoints ?? []).map(({ category, startingPoints }) => (
              <section className="open-category" key={`${category.en} ${category.zh}`}>
                <h3 className="open-category-title">
                  <BilingualTitleText title={category} />
                </h3>
                <div className="open-choices">
                  {startingPoints.map((startingPoint) => (
                    <button type="button" key={startingPoint.name} className="open-choice" onClick={() => props.onOpen({ source: 'starting_point', startingPoint })}>
                      <span className="choice-title">
                        <BilingualTitleText title={startingPoint.title} />
                      </span>
                      <span className="choice-description">
                        <BilingualTitleText title={startingPoint.description} />
                      </span>
                      <span className="choice-tag">{startingPoint.name}</span>
                    </button>
                  ))}
                </div>
              </section>
            ))}
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
