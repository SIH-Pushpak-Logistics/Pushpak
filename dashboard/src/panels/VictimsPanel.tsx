import React from 'react';
import type { DashboardState } from '../types';
import { Crosshair, MapPin } from 'lucide-react';

interface Props {
  state: DashboardState;
}

export const VictimsPanel: React.FC<Props> = ({ state }) => {
  return (
    <div className="panel">
      <div className="panel-header">
        <div className="panel-title">
          <Crosshair size={16} /> Victim Alerts
        </div>
      </div>
      <div className="victim-list">
        {state.victims.length === 0 ? (
          <div style={{ color: 'var(--text-muted)', fontSize: '0.875rem', textAlign: 'center', padding: '1rem 0' }}>
            No victims detected.
          </div>
        ) : (
          state.victims.map(v => (
            <div key={v.victim_id} className="victim-card">
              <div className="victim-header">
                <span className="victim-id">{v.victim_id}</span>
                <span className="victim-conf">{(v.confidence * 100).toFixed(0)}% Conf</span>
              </div>
              <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                <span className="victim-loc" style={{ display: 'flex', alignItems: 'center', gap: '4px' }}>
                  <MapPin size={12} /> {v.world_x.toFixed(2)}, {v.world_y.toFixed(2)}
                </span>
                <span className="status-badge" style={{ fontSize: '0.65rem', border: v.acked ? '1px solid var(--status-online)' : '1px solid var(--status-offline)' }}>
                  {v.acked ? 'ACKED' : 'UNACKED'}
                </span>
              </div>
            </div>
          ))
        )}
      </div>
    </div>
  );
};
