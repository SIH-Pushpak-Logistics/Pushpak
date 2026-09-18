import React from 'react';
import type { DashboardState } from '../types';
import { Activity, AlertTriangle, Navigation } from 'lucide-react';

interface Props {
  state: DashboardState;
}

export const LiveTelemetry: React.FC<Props> = ({ state }) => {
  const { pose, altitude, velocity } = state;
  const isValid = velocity?.is_valid !== false; // defaults to true if undefined

  return (
    <div className="panel">
      <div className="panel-header">
        <div className="panel-title">
          <Activity size={16} /> Live Telemetry
        </div>
      </div>

      {!isValid && (
        <div className="warning-banner">
          <AlertTriangle size={18} /> Vision Degraded
        </div>
      )}

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

      <div className="panel-header" style={{ marginTop: '0.5rem', border: 'none' }}>
        <div className="panel-title">
          <Navigation size={16} /> Velocity (Body Frame)
        </div>
      </div>
      
      <div className="metric-grid">
        <div className="metric-item">
          <div className="metric-label">Linear X</div>
          <div className={`metric-value mono ${!isValid ? 'disabled' : ''}`}>
            {velocity ? velocity.linear_x.toFixed(2) : '--'} m/s
          </div>
        </div>
        <div className="metric-item">
          <div className="metric-label">Linear Y</div>
          <div className={`metric-value mono ${!isValid ? 'disabled' : ''}`}>
            {velocity ? velocity.linear_y.toFixed(2) : '--'} m/s
          </div>
        </div>
        <div className="metric-item">
          <div className="metric-label">Features</div>
          <div className="metric-value mono">
            {velocity ? velocity.features : '--'}
          </div>
        </div>
      </div>
    </div>
  );
};
