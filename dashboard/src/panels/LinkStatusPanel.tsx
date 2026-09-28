import React from 'react';
import type { DashboardState } from '../types';
import { Radio } from 'lucide-react';

interface Props {
  state: DashboardState;
}

export const LinkStatusPanel: React.FC<Props> = ({ state }) => {
  const { linkStatus } = state;
  const statusType = linkStatus?.state || 'OFFLINE';
  const rssi = linkStatus?.rssi_dbm;
  const lastSync = linkStatus?.last_sync_sec;

  return (
    <div className="panel">
      <div className="panel-header">
        <div className="panel-title">
          <Radio size={16} /> Link Status
        </div>
      </div>
      
      <div className={`link-status-banner ${statusType}`}>
        {statusType}
      </div>

      <div className="metric-grid">
        <div className="metric-item">
          <div className="metric-label">RSSI</div>
          <div className="metric-value mono">
            {typeof rssi === 'number' ? `${rssi.toFixed(1)} dBm` : 'not measured'}
          </div>
        </div>
        <div className="metric-item">
          <div className="metric-label">Cached Pkts</div>
          <div className="metric-value mono">
            {linkStatus ? linkStatus.cached_packets : '--'}
          </div>
        </div>
        <div className="metric-item" style={{ gridColumn: '1 / -1' }}>
          <div className="metric-label">Last Heartbeat</div>
          <div className="metric-value mono" style={{ fontSize: '1rem' }}>
            {typeof lastSync === 'number' ? `${lastSync.toFixed(2)}s ago` : '--'}
          </div>
        </div>
      </div>
    </div>
  );
};
