import { useState } from 'react';
import { useTelemetry } from './hooks/useTelemetry';
import { LiveTelemetry } from './panels/LiveTelemetry';
import { MapPanel } from './panels/MapPanel';
import { VictimsPanel } from './panels/VictimsPanel';
import { LinkStatusPanel } from './panels/LinkStatusPanel';

const WS_URL = import.meta.env.VITE_WS_URL || `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.hostname}:8765`;

export default function App() {
  const [selectedId, setSelectedId] = useState('1');
  const { state, fleet, connected } = useTelemetry(WS_URL, selectedId);

  return (
    <div className="app-container">
      <header className="app-header">
        <div className="app-header-left">
          <h1>PUSHPAK UAV OPERATIONS</h1>
        </div>
        <div className="app-header-right">
          <div className="status-badge">
            <div className={`status-dot ${connected ? 'connected' : 'disconnected'}`} />
            {connected ? 'GROUND LINK CONNECTED' : 'GROUND LINK OFFLINE'}
          </div>
          <div className="status-badge" style={{ background: 'transparent', border: 'none', padding: 0 }}>
            <label htmlFor="drone-select">Drone </label>
            <select id="drone-select" value={selectedId}
              onChange={event => setSelectedId(event.target.value)}>
              {Object.keys(fleet).length ? Object.keys(fleet).map(id =>
                <option key={id} value={id}>Drone {id}</option>) :
                <><option value="1">Drone 1</option><option value="2">Drone 2</option></>}
            </select>
          </div>
        </div>
      </header>

      <div aria-label="Vehicle link status" style={{ display: 'flex', gap: '0.75rem', padding: '0.5rem 1rem' }}>
        {Object.entries(fleet).map(([id, vehicle]) =>
          <button key={id} type="button" onClick={() => setSelectedId(id)}
            aria-pressed={selectedId === id} style={{ cursor: 'pointer' }}>
            Drone {id}: {vehicle.linkStatus?.state ?? 'OFFLINE'}
          </button>)}
      </div>

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
