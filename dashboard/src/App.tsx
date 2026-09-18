import { useTelemetry } from './hooks/useTelemetry';
import { LiveTelemetry } from './panels/LiveTelemetry';
import { MapPanel } from './panels/MapPanel';
import { VictimsPanel } from './panels/VictimsPanel';
import { LinkStatusPanel } from './panels/LinkStatusPanel';

const WS_URL = 'ws://localhost:8765';

export default function App() {
  const state = useTelemetry(WS_URL);

  return (
    <div className="app-container">
      <header className="app-header">
        <div className="app-header-left">
          <h1>PUSHPAK UAV OPERATIONS</h1>
        </div>
        <div className="app-header-right">
          <div className="status-badge">
            <div className={`status-dot ${state.connected ? 'connected' : 'disconnected'}`} />
            {state.connected ? 'DATA LINK CONNECTED' : 'DATA LINK OFFLINE'}
          </div>
          <div className="status-badge" style={{ background: 'transparent', border: 'none', padding: 0 }}>
            DRONE-00
          </div>
        </div>
      </header>

      <div className="dashboard-grid">
        <div className="left-column">
          <LiveTelemetry state={state} />
        </div>
        
        <div className="center-column">
          <MapPanel state={state} />
        </div>
        
        <div className="right-column">
          <LinkStatusPanel state={state} />
          <VictimsPanel state={state} />
        </div>
      </div>
    </div>
  );
}
