import React from 'react';
import type { DashboardState } from '../types';
import { Activity } from 'lucide-react';

interface Props {
  state: DashboardState;
}

export const LiveTelemetry: React.FC<Props> = ({ state }) => {
  const { pose, altitude } = state;

  return (
    <div className="panel">
      <div className="panel-header">
        <div className="panel-title">
          <Activity size={16} /> Live Telemetry (estimate)
        </div>
      </div>

      <div className="metric-grid">
        <div className="metric-item">
          <div className="metric-label">X Position</div>
          <div className="metric-value mono">
            {pose ? pose.x.toFixed(2) : '--'} m
          </div>
        </div>
        <div className="metric-item">
          <div className="metric-label">Y Position</div>
          <div className="metric-value mono">
            {pose ? pose.y.toFixed(2) : '--'} m
          </div>
        </div>
        <div className="metric-item">
          <div className="metric-label">Altitude AGL</div>
          <div className="metric-value mono">
            {altitude ? altitude.z.toFixed(2) : (pose ? pose.z.toFixed(2) : '--')} m
          </div>
        </div>
        <div className="metric-item">
          <div className="metric-label">Heading (Yaw)</div>
          <div className="metric-value mono">
            {pose ? pose.yaw_deg.toFixed(1) : '--'}°
          </div>
        </div>
      </div>
    </div>
  );
};
