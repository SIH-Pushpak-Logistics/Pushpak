import React, { useRef, useEffect, useState } from 'react';
import type { DashboardState } from '../types';
import { Map } from 'lucide-react';

interface Props {
  state: DashboardState;
}

export const MapPanel: React.FC<Props> = ({ state }) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const [dimensions, setDimensions] = useState({ width: 0, height: 0 });

  // Handle Resize
  useEffect(() => {
    if (!containerRef.current) return;
    const observer = new ResizeObserver((entries) => {
      for (let entry of entries) {
        setDimensions({
          width: entry.contentRect.width,
          height: entry.contentRect.height
        });
      }
    });
    observer.observe(containerRef.current);
    return () => observer.disconnect();
  }, []);

  // Draw Map
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || dimensions.width === 0) return;
    
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // High DPI Canvas setup
    const dpr = window.devicePixelRatio || 1;
    canvas.width = dimensions.width * dpr;
    canvas.height = dimensions.height * dpr;
    ctx.scale(dpr, dpr);

    const { width, height } = dimensions;
    
    // Clear and draw grid
    ctx.clearRect(0, 0, width, height);
    ctx.strokeStyle = '#27272a';
    ctx.lineWidth = 1;
    
    const gridSize = 50;
    for (let x = 0; x < width; x += gridSize) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, height);
        ctx.stroke();
    }
    for (let y = 0; y < height; y += gridSize) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();
    }

    // Auto-scaling logic (center map based on track + victims)
    let minX = -10, maxX = 10, minY = -10, maxY = 10;
    
    if (state.pose) {
        minX = Math.min(minX, state.pose.x - 5);
        maxX = Math.max(maxX, state.pose.x + 5);
        minY = Math.min(minY, state.pose.y - 5);
        maxY = Math.max(maxY, state.pose.y + 5);
    }
    
    state.victims.forEach(v => {
        minX = Math.min(minX, v.world_x - 5);
        maxX = Math.max(maxX, v.world_x + 5);
        minY = Math.min(minY, v.world_y - 5);
        maxY = Math.max(maxY, v.world_y + 5);
    });

    const rangeX = maxX - minX;
    const rangeY = maxY - minY;
    
    // Scale factor (pixels per meter)
    const scale = Math.min((width - 40) / rangeX, (height - 40) / rangeY);
    
    const cx = width / 2;
    const cy = height / 2;
    const midX = (minX + maxX) / 2;
    const midY = (minY + maxY) / 2;

    // Helper to map ENU to Screen:
    // ENU: X right, Y up
    // Screen: X right, Y down
    const toScreen = (x: number, y: number) => {
        const sx = cx + (x - midX) * scale;
        const sy = cy - (y - midY) * scale;
        return { x: sx, y: sy };
    };

    // Draw Track
    if (state.track.length > 1) {
        ctx.beginPath();
        ctx.strokeStyle = 'rgba(59, 130, 246, 0.5)';
        ctx.lineWidth = 2;
        const first = toScreen(state.track[0].x, state.track[0].y);
        ctx.moveTo(first.x, first.y);
        for (let i = 1; i < state.track.length; i++) {
            const p = toScreen(state.track[i].x, state.track[i].y);
            ctx.lineTo(p.x, p.y);
        }
        ctx.stroke();
    }

    // Draw Victims
    state.victims.forEach(v => {
        const p = toScreen(v.world_x, v.world_y);
        ctx.beginPath();
        ctx.fillStyle = v.acked ? '#22c55e' : '#ef4444';
        ctx.arc(p.x, p.y, 6, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 1.5;
        ctx.stroke();

        ctx.fillStyle = '#fff';
        ctx.font = '10px Inter';
        ctx.textAlign = 'center';
        ctx.fillText(v.victim_id, p.x, p.y - 12);
    });

    // Draw Drone
    if (state.pose) {
        const p = toScreen(state.pose.x, state.pose.y);
        const yawRad = (state.pose.yaw_deg * Math.PI) / 180;
        
        ctx.save();
        ctx.translate(p.x, p.y);
        // ENU yaw: 0 is East, 90 is North. 
        // Screen Canvas: 0 is Right, 90 is Down.
        // So we need to invert yaw to match screen rotation
        ctx.rotate(-yawRad);
        
        ctx.beginPath();
        ctx.fillStyle = '#3b82f6';
        // Triangle pointing right (which is 0 degrees / East)
        ctx.moveTo(12, 0);
        ctx.lineTo(-8, -8);
        ctx.lineTo(-4, 0);
        ctx.lineTo(-8, 8);
        ctx.closePath();
        ctx.fill();
        
        // Add a subtle glow
        ctx.shadowColor = '#3b82f6';
        ctx.shadowBlur = 10;
        ctx.fill();
        
        ctx.restore();
    }
  }, [state, dimensions]);

  return (
    <div className="panel" style={{ flex: 1, padding: 0, border: 'none', background: 'transparent' }}>
      <div className="panel-header" style={{ padding: '1rem', background: 'var(--bg-panel)', borderBottom: '1px solid var(--border)' }}>
        <div className="panel-title">
          <Map size={16} /> 2D Map & Track
        </div>
      </div>
      <div className="map-container" ref={containerRef}>
        <canvas ref={canvasRef} style={{ width: '100%', height: '100%', display: 'block' }} />
      </div>
    </div>
  );
};
