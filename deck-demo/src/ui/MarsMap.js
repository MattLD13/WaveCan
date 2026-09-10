import { CHECKPOINTS, MISSION_ROUTE } from '../sim/MissionState.js';

const COLORS = {
  bg: '#120f14',
  grid: '#302a35',
  contour: '#57403e',
  contourBright: '#7b5146',
  route: '#a98c80',
  routeVisited: '#d5b0a0',
  purple: '#b78dff',
  dim: '#978e9d',
  ink: '#f3edf5',
  marker: '#e29b61'
};

const byPoint = (point, width, height, pad = 10) => ({ x: pad + point.x * (width - pad * 2), y: pad + point.y * (height - pad * 2) });

export class MarsMap {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.lastDpr = 1;
    this.resize();
    window.addEventListener('resize', () => this.resize());
  }

  resize() {
    if (!this.canvas || !this.ctx) return;
    const width = Math.max(1, this.canvas.clientWidth || 220);
    const height = Math.max(1, this.canvas.clientHeight || 130);
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    this.lastDpr = dpr;
    this.canvas.width = Math.round(width * dpr);
    this.canvas.height = Math.round(height * dpr);
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.width = width;
    this.height = height;
  }

  _terrain(width, height, detail = false) {
    const ctx = this.ctx;
    ctx.fillStyle = COLORS.bg;
    ctx.fillRect(0, 0, width, height);
    ctx.strokeStyle = COLORS.grid;
    ctx.lineWidth = 0.6;
    for (let x = 0; x <= width; x += 22) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, height); ctx.stroke(); }
    for (let y = 0; y <= height; y += 22) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke(); }
    const contours = [
      [[0.04, 0.73], [0.19, 0.66], [0.36, 0.73], [0.54, 0.62], [0.72, 0.69], [0.96, 0.57]],
      [[0.02, 0.48], [0.2, 0.4], [0.35, 0.49], [0.49, 0.37], [0.7, 0.44], [0.98, 0.32]],
      [[0.03, 0.25], [0.18, 0.19], [0.36, 0.29], [0.52, 0.2], [0.72, 0.27], [0.96, 0.16]]
    ];
    ctx.strokeStyle = COLORS.contour;
    ctx.lineWidth = 1;
    for (const contour of contours) {
      ctx.beginPath();
      contour.forEach((point, index) => { const mapped = byPoint({ x: point[0], y: point[1] }, width, height, 8); if (index === 0) ctx.moveTo(mapped.x, mapped.y); else ctx.lineTo(mapped.x, mapped.y); });
      ctx.stroke();
    }
    ctx.strokeStyle = COLORS.contourBright;
    ctx.beginPath();
    for (let i = 0; i <= 40; i += 1) { const x = i / 40; const y = 0.53 - 0.08 * Math.sin(x * 8) - 0.09 * x; const mapped = byPoint({ x, y }, width, height, 8); if (i === 0) ctx.moveTo(mapped.x, mapped.y); else ctx.lineTo(mapped.x, mapped.y); }
    ctx.stroke();
    if (detail) {
      ctx.strokeStyle = '#6d4c48';
      ctx.lineWidth = 0.55;
      for (let row = 0; row < 7; row += 1) {
        ctx.beginPath();
        for (let column = 0; column <= 32; column += 1) {
          const x = column / 32;
          const y = 0.12 + row * 0.115 + 0.018 * Math.sin(x * 14 + row * 1.7);
          const mapped = byPoint({ x, y }, width, height, 8);
          if (column === 0) ctx.moveTo(mapped.x, mapped.y); else ctx.lineTo(mapped.x, mapped.y);
        }
        ctx.stroke();
      }
      ctx.fillStyle = '#8c5e51';
      for (let index = 0; index < 18; index += 1) {
        const x = 0.08 + ((index * 37) % 83) / 100;
        const y = 0.14 + ((index * 19) % 68) / 100;
        const mapped = byPoint({ x, y }, width, height, 8);
        ctx.beginPath(); ctx.arc(mapped.x, mapped.y, 0.9 + (index % 3) * 0.35, 0, Math.PI * 2); ctx.fill();
      }
    }
  }

  _route(state, width, height) {
    const ctx = this.ctx;
    const route = state.mission?.route || MISSION_ROUTE;
    ctx.lineWidth = 2.2;
    ctx.lineJoin = 'round';
    ctx.strokeStyle = COLORS.route;
    ctx.beginPath();
    route.forEach((point, index) => { const mapped = byPoint(point, width, height, 8); if (index === 0) ctx.moveTo(mapped.x, mapped.y); else ctx.lineTo(mapped.x, mapped.y); });
    ctx.stroke();
    const progressIndex = state.mission?.phase === 'drive-outcrop' ? 0 : state.mission?.phase === 'science' ? 3 : state.mission?.phase === 'drive-marker' ? 6 : state.mission?.phase === 'beacon' ? 6 : 8;
    ctx.strokeStyle = COLORS.routeVisited;
    ctx.beginPath();
    route.slice(0, progressIndex + 1).forEach((point, index) => { const mapped = byPoint(point, width, height, 8); if (index === 0) ctx.moveTo(mapped.x, mapped.y); else ctx.lineTo(mapped.x, mapped.y); });
    ctx.stroke();
    // Small directional chevrons make the route direction legible without a
    // generic radar sweep or a game-style compass overlay.
    ctx.fillStyle = COLORS.route;
    for (let index = 1; index < route.length - 1; index += 2) {
      const current = byPoint(route[index], width, height, 8);
      const next = byPoint(route[index + 1], width, height, 8);
      const angle = Math.atan2(next.y - current.y, next.x - current.x);
      ctx.save(); ctx.translate(current.x, current.y); ctx.rotate(angle);
      ctx.beginPath(); ctx.moveTo(5, 0); ctx.lineTo(-3, -3); ctx.lineTo(-1, 0); ctx.lineTo(-3, 3); ctx.closePath(); ctx.fill(); ctx.restore();
    }
  }

  _checkpoint(point, visited, current, width, height, label) {
    const ctx = this.ctx;
    const mapped = byPoint(point, width, height, 8);
    ctx.beginPath(); ctx.arc(mapped.x, mapped.y, current ? 7 : 4, 0, Math.PI * 2);
    ctx.fillStyle = visited ? COLORS.routeVisited : COLORS.marker; ctx.fill();
    ctx.lineWidth = current ? 1.4 : 0.8; ctx.strokeStyle = current ? COLORS.purple : COLORS.marker; ctx.stroke();
    if (current || visited) {
      ctx.font = '600 7px ui-monospace, monospace'; ctx.fillStyle = COLORS.ink; ctx.textAlign = mapped.x > width * 0.68 ? 'right' : 'left';
      ctx.fillText(label, mapped.x > width * 0.68 ? mapped.x - 9 : mapped.x + 9, mapped.y - 7);
    }
  }

  render(state) {
    if (!this.ctx) return;
    const width = this.width || this.canvas.clientWidth || 220;
    const height = this.height || this.canvas.clientHeight || 130;
    this._terrain(width, height, Boolean(state.mapDetail));
    this._route(state, width, height);
    const visited = state.mission?.visited || {};
    this._checkpoint(CHECKPOINTS.start, visited.start, state.mission?.currentCheckpoint === 'start', width, height, 'BASE');
    this._checkpoint(CHECKPOINTS.outcrop, visited.outcrop, state.mission?.currentCheckpoint === 'outcrop', width, height, 'OUTCROP');
    this._checkpoint(CHECKPOINTS.marker, visited.marker, state.mission?.currentCheckpoint === 'marker', width, height, 'MARKER');
    this._checkpoint(CHECKPOINTS.final, visited.final, state.mission?.currentCheckpoint === 'final', width, height, 'FINAL');
    const rover = byPoint({ x: state.pose.x, y: state.pose.y }, width, height, 8);
    const angle = state.pose.heading * Math.PI / 180;
    const ctx = this.ctx;
    ctx.save(); ctx.translate(rover.x, rover.y); ctx.rotate(angle);
    ctx.fillStyle = COLORS.purple; ctx.strokeStyle = '#f3edf5'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(8, 0); ctx.lineTo(-6, -4); ctx.lineTo(-6, 4); ctx.closePath(); ctx.fill(); ctx.stroke(); ctx.restore();
    ctx.font = '600 7px ui-monospace, monospace'; ctx.fillStyle = COLORS.dim; ctx.textAlign = 'left'; ctx.fillText('TERRAIN ROUTE', 8, height - 7);
  }
}
