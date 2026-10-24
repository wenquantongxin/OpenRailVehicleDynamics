import type { LiveController } from '../live/live_controller.ts';
import type { LiveConnectionStatus } from '../live/live_connection.ts';
import type { SceneSampleView } from '../record/scene_sample.ts';
import { Card } from './Card.tsx';
import { fixed } from './format.ts';

export function LiveCard({ controller, status, sample, timeSeconds, onToggle }: {
  controller: LiveController; status: LiveConnectionStatus; sample: SceneSampleView; timeSeconds: number; onToggle: () => void;
}) {
  const identity = sample.identity;
  const label = status.state === 'error' ? 'Stream error · 数据流错误' :
    status.state === 'reconnecting' ? 'Reconnecting · 正在重连' :
    status.state === 'connecting' ? 'Connecting · 正在连接' :
    status.state === 'loading' ? 'Loading scene · 正在载入场景' :
    controller.isStale ? 'Waiting for states · 等待状态' : controller.isPaused ? 'Paused · 已暂停显示' : 'Live · 实时';
  return <Card index="07" en="Live" zh="实时" className="playback live-card">
    <div className="live-controls">
      <button type="button" className="chip on" onClick={onToggle}>{controller.isPaused ? 'Return live 返回实时' : 'Pause display 暂停显示'}</button>
      <strong role="status">{label}</strong>
      <span className="live-clock">{fixed(timeSeconds, 3)} s <span className="zh">仿真时间</span></span>
    </div>
    {identity.kind === 'stream' && <div className="live-identity">Run {identity.runId.toString()} · Message {identity.sequence.toString()}</div>}
    {status.message !== '' && <p className="live-error">{status.message}</p>}
  </Card>;
}
