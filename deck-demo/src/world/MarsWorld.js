import * as THREE from '../../node_modules/three/build/three.module.js';

const MAP_WORLD_SIZE = 82;
const TERRAIN_SEGMENTS = 96;
const MARS_COLORS = {
  sand: new THREE.Color('#8d5540'),
  sandLight: new THREE.Color('#b87753'),
  sandDark: new THREE.Color('#55362f'),
  rock: new THREE.Color('#5a3a35'),
  rockWarm: new THREE.Color('#7c4b3a'),
  track: new THREE.Color('#4b302e'),
  flag: new THREE.Color('#b76f43'),
  beacon: new THREE.Color('#b78dff')
};

const seeded = (seed) => {
  let value = seed >>> 0;
  return () => {
    value += 0x6D2B79F5;
    let t = value;
    t = Math.imul(t ^ t >>> 15, t | 1);
    t ^= t + Math.imul(t ^ t >>> 7, t | 61);
    return ((t ^ t >>> 14) >>> 0) / 4294967296;
  };
};

const terrainHeight = (x, z) => {
  // A layered, deterministic height field keeps the world light enough for a
  // Steam Deck while giving the FPV camera a genuine ridge, basin, and slope.
  const ridge = 1.8 * Math.exp(-((z + 19) ** 2) / 145) * (0.45 + 0.55 * Math.cos(x * 0.11));
  const shelf = 0.75 * Math.exp(-((x - 20) ** 2 + (z + 3) ** 2) / 230);
  const broad = 0.44 * Math.sin(x * 0.12 + 0.5) + 0.34 * Math.cos(z * 0.16 - 0.7);
  const small = 0.17 * Math.sin(x * 0.51 + z * 0.37) + 0.1 * Math.cos(x * 0.29 - z * 0.67);
  return -0.35 + ridge + shelf + broad + small;
};

const mapToWorld = (x, y) => ({ x: (x - 0.5) * MAP_WORLD_SIZE, z: (0.5 - y) * MAP_WORLD_SIZE });

const makeTerrain = () => {
  const size = MAP_WORLD_SIZE;
  const segments = TERRAIN_SEGMENTS;
  const positions = [];
  const colors = [];
  const indices = [];
  for (let row = 0; row <= segments; row += 1) {
    const z = (row / segments - 0.5) * size;
    for (let column = 0; column <= segments; column += 1) {
      const x = (column / segments - 0.5) * size;
      const height = terrainHeight(x, z);
      positions.push(x, height, z);
      const noise = Math.sin(x * 0.31 + z * 0.18) * 0.5 + 0.5;
      const color = MARS_COLORS.sand.clone().lerp(noise > 0.66 ? MARS_COLORS.sandLight : MARS_COLORS.sandDark, noise * 0.24);
      if (height > 1.15) color.lerp(MARS_COLORS.rockWarm, Math.min(0.38, (height - 1.15) * 0.14));
      colors.push(color.r, color.g, color.b);
    }
  }
  for (let row = 0; row < segments; row += 1) {
    for (let column = 0; column < segments; column += 1) {
      const a = row * (segments + 1) + column;
      const b = a + 1;
      const c = a + segments + 1;
      const d = c + 1;
      indices.push(a, c, b, b, c, d);
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  const material = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 1, metalness: 0, flatShading: true });
  return new THREE.Mesh(geometry, material);
};

const makeRock = (random, scale = 1, dark = false) => {
  const geometry = new THREE.DodecahedronGeometry(1, 1);
  const material = new THREE.MeshStandardMaterial({
    color: dark ? MARS_COLORS.rock : MARS_COLORS.rockWarm,
    roughness: 0.98,
    metalness: 0
  });
  const rock = new THREE.Mesh(geometry, material);
  rock.scale.set(scale * (0.75 + random() * 0.55), scale * (0.55 + random() * 0.65), scale * (0.8 + random() * 0.65));
  rock.rotation.set(random() * 0.5, random() * Math.PI, random() * 0.45);
  return rock;
};

const makeTrack = (a, b, offset) => {
  const start = mapToWorld(a.x, a.y);
  const end = mapToWorld(b.x, b.y);
  const direction = new THREE.Vector2(end.x - start.x, end.z - start.z);
  const length = direction.length();
  direction.normalize();
  const perpendicular = new THREE.Vector2(-direction.y, direction.x).multiplyScalar(offset);
  const center = new THREE.Vector3((start.x + end.x) / 2 + perpendicular.x, 0, (start.z + end.z) / 2 + perpendicular.y);
  const track = new THREE.Mesh(
    new THREE.BoxGeometry(0.16, 0.035, length),
    new THREE.MeshStandardMaterial({ color: MARS_COLORS.track, roughness: 1 })
  );
  track.position.set(center.x, terrainHeight(center.x, center.z) + 0.015, center.z);
  track.rotation.y = Math.atan2(direction.x, direction.y);
  return track;
};

const makeMarker = (position, isFinal = false) => {
  const group = new THREE.Group();
  group.position.set(position.x, terrainHeight(position.x, position.z), position.z);
  const pole = new THREE.Mesh(
    new THREE.CylinderGeometry(0.06, 0.08, 2.1, 8),
    new THREE.MeshStandardMaterial({ color: isFinal ? MARS_COLORS.beacon : MARS_COLORS.flag, roughness: 0.82 })
  );
  pole.position.y = 1.05;
  group.add(pole);
  const plate = new THREE.Mesh(
    new THREE.BoxGeometry(0.8, 0.48, 0.06),
    new THREE.MeshStandardMaterial({ color: isFinal ? '#4c326c' : '#6f4c3c', roughness: 0.86 })
  );
  plate.position.set(0, 1.72, 0);
  group.add(plate);
  const stripe = new THREE.Mesh(
    new THREE.BoxGeometry(0.55, 0.05, 0.07),
    new THREE.MeshStandardMaterial({ color: isFinal ? '#c2a9ff' : '#d89959', emissive: isFinal ? '#3d2061' : '#4a1d12', emissiveIntensity: 0.4 })
  );
  stripe.position.set(0, 1.72, 0.04);
  group.add(stripe);
  if (isFinal) {
    const light = new THREE.PointLight('#b78dff', 1.15, 7, 2);
    light.position.set(0, 2, 0);
    group.add(light);
  }
  return group;
};

const makeRover = () => {
  const group = new THREE.Group();
  const bodyMaterial = new THREE.MeshStandardMaterial({ color: '#242b30', roughness: 0.78, metalness: 0.35 });
  const edgeMaterial = new THREE.MeshStandardMaterial({ color: '#756a63', roughness: 0.65, metalness: 0.55 });
  const body = new THREE.Mesh(new THREE.BoxGeometry(2.05, 0.48, 2.65), bodyMaterial);
  body.position.y = 0.58;
  group.add(body);
  const deck = new THREE.Mesh(new THREE.BoxGeometry(1.75, 0.12, 2.1), edgeMaterial);
  deck.position.y = 0.88;
  group.add(deck);
  const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.1, 1.4, 8), edgeMaterial);
  mast.position.set(0, 1.55, 0.38);
  group.add(mast);
  const cameraHead = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.28, 0.3), new THREE.MeshStandardMaterial({ color: '#11181c', metalness: 0.58, roughness: 0.4 }));
  cameraHead.position.set(0, 2.16, 0.38);
  group.add(cameraHead);
  const wheelMaterial = new THREE.MeshStandardMaterial({ color: '#151b1e', roughness: 0.95, metalness: 0.12 });
  for (const side of [-1, 1]) {
    for (const z of [-0.86, 0, 0.86]) {
      const wheel = new THREE.Mesh(new THREE.CylinderGeometry(0.44, 0.44, 0.22, 12), wheelMaterial);
      wheel.rotation.z = Math.PI / 2;
      wheel.position.set(side * 1.06, 0.42, z);
      group.add(wheel);
    }
  }
  // The short arm is visible in ARM and OVERHEAD cameras and stays local to the
  // simulated rover model. It never maps to an actuator or device.
  const armMaterial = new THREE.MeshStandardMaterial({ color: '#a18b7b', roughness: 0.65, metalness: 0.35 });
  const armBase = new THREE.Mesh(new THREE.CylinderGeometry(0.3, 0.35, 0.28, 12), armMaterial);
  armBase.position.set(0.7, 1.18, -0.4);
  group.add(armBase);
  const armLink = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.18, 1.15), armMaterial);
  armLink.position.set(0.65, 1.55, -0.88);
  armLink.rotation.x = -0.45;
  group.add(armLink);
  const scoop = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.24, 0.44), edgeMaterial);
  scoop.position.set(0.61, 1.6, -1.35);
  scoop.rotation.x = -0.3;
  group.add(scoop);
  return group;
};

const makeDust = () => {
  const random = seeded(91427);
  const points = [];
  for (let i = 0; i < 900; i += 1) {
    const x = (random() - 0.5) * MAP_WORLD_SIZE;
    const z = (random() - 0.5) * MAP_WORLD_SIZE;
    points.push(x, terrainHeight(x, z) + 0.08 + random() * 0.18, z);
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(points, 3));
  const material = new THREE.PointsMaterial({ color: '#c48969', size: 0.07, sizeAttenuation: true, transparent: true, opacity: 0.34 });
  return new THREE.Points(geometry, material);
};

export class MarsWorld {
  constructor(canvas) {
    this.canvas = canvas;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color('#6f4c47');
    this.scene.fog = new THREE.Fog('#6f4c47', 32, 115);
    this.camera = new THREE.PerspectiveCamera(64, 1, 0.05, 160);
    this.camera.rotation.order = 'YXZ';
    this.renderer = null;
    this.fallback = false;
    try {
      this.renderer = new THREE.WebGLRenderer({ canvas, antialias: false, powerPreference: 'high-performance' });
      this.renderer.outputColorSpace = THREE.SRGBColorSpace;
      this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
      this.renderer.toneMappingExposure = 1.08;
      this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.35));
    } catch (error) {
      // A software-only browser test may not expose WebGL. Keep the app alive
      // and show a quiet canvas fallback; packaged Steam Deck uses WebGL.
      this.fallback = true;
      this.fallbackContext = canvas.getContext('2d');
      console.warn('SIM WEBGL FALLBACK', error?.message || error);
    }
    this._buildScene();
    this.resize();
    window.addEventListener('resize', () => this.resize());
  }

  _buildScene() {
    const hemisphere = new THREE.HemisphereLight('#f0c5a4', '#2c2021', 1.55);
    this.scene.add(hemisphere);
    const sun = new THREE.DirectionalLight('#ffd1ad', 2.15);
    sun.position.set(-28, 42, 19);
    this.scene.add(sun);
    this.scene.add(makeTerrain());
    const random = seeded(30101);
    const scatter = new THREE.Group();
    for (let i = 0; i < 76; i += 1) {
      const x = (random() - 0.5) * (MAP_WORLD_SIZE - 6);
      const z = (random() - 0.5) * (MAP_WORLD_SIZE - 6);
      // Leave the marked route readable from the camera.
      if (Math.abs(x + z * 0.6) < 3.4) continue;
      const scale = 0.16 + random() * 0.62;
      const rock = makeRock(random, scale, random() > 0.64);
      rock.position.set(x, terrainHeight(x, z) + scale * 0.34, z);
      scatter.add(rock);
    }
    this.scene.add(scatter);
    this.scene.add(makeDust());

    // Broad, low-poly horizon mounds give the forward feed a readable ridge
    // line without importing a texture or paying for a heavy asset.
    const ridgeMaterial = new THREE.MeshStandardMaterial({ color: '#5c3c37', roughness: 1, flatShading: true });
    const ridge = new THREE.Group();
    for (const [x, z, sx, sy, sz] of [[-25, 14, 11, 2.8, 5], [-10, 17, 14, 3.7, 6], [10, 15, 18, 3.2, 6], [29, 18, 12, 2.7, 5]]) {
      const mound = new THREE.Mesh(new THREE.SphereGeometry(1, 16, 7), ridgeMaterial);
      mound.position.set(x, terrainHeight(x, z) + sy * 0.38, z);
      mound.scale.set(sx, sy, sz);
      ridge.add(mound);
    }
    this.scene.add(ridge);

    const route = [
      { x: 0.12, y: 0.78 }, { x: 0.21, y: 0.66 }, { x: 0.31, y: 0.59 }, { x: 0.44, y: 0.47 },
      { x: 0.56, y: 0.39 }, { x: 0.64, y: 0.34 }, { x: 0.73, y: 0.27 }, { x: 0.82, y: 0.2 }, { x: 0.89, y: 0.12 }
    ];
    const tracks = new THREE.Group();
    for (let i = 0; i < route.length - 1; i += 1) {
      tracks.add(makeTrack(route[i], route[i + 1], -0.72));
      tracks.add(makeTrack(route[i], route[i + 1], 0.72));
    }
    this.scene.add(tracks);

    const outcrop = mapToWorld(0.44, 0.47);
    const outcropGroup = new THREE.Group();
    outcropGroup.position.set(outcrop.x, terrainHeight(outcrop.x, outcrop.z), outcrop.z);
    const outcropRock = makeRock(seeded(7001), 2.1, true);
    outcropRock.position.set(0, 1.1, 0);
    outcropRock.scale.multiply(new THREE.Vector3(1.35, 0.8, 1.05));
    outcropGroup.add(outcropRock);
    for (let i = 0; i < 6; i += 1) {
      const rock = makeRock(seeded(7120 + i), 0.4 + i * 0.1, i % 2 === 0);
      rock.position.set(-1.55 + (i % 3) * 1.3, 0.25 + (i % 2) * 0.12, -0.9 + Math.floor(i / 3) * 1.25);
      outcropGroup.add(rock);
    }
    this.scene.add(outcropGroup);
    this.outcrop = outcropGroup;

    const markerPosition = mapToWorld(0.73, 0.27);
    this.marker = makeMarker(markerPosition, false);
    this.scene.add(this.marker);
    const finalPosition = mapToWorld(0.89, 0.12);
    this.finalMarker = makeMarker(finalPosition, true);
    this.scene.add(this.finalMarker);
    this.rover = makeRover();
    this.scene.add(this.rover);
    this.target = new THREE.Vector3();
    this.currentTarget = new THREE.Vector3();
    this.currentPosition = new THREE.Vector3();
  }

  resize() {
    const width = Math.max(1, this.canvas.clientWidth || this.canvas.width || 1);
    const height = Math.max(1, this.canvas.clientHeight || this.canvas.height || 1);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    if (this.renderer) this.renderer.setSize(width, height, false);
  }

  _fallback(state) {
    const ctx = this.fallbackContext;
    if (!ctx) return;
    const width = this.canvas.width = this.canvas.clientWidth || 800;
    const height = this.canvas.height = this.canvas.clientHeight || 500;
    const gradient = ctx.createLinearGradient(0, 0, 0, height);
    gradient.addColorStop(0, '#855c58'); gradient.addColorStop(0.48, '#a96850'); gradient.addColorStop(1, '#3d2929');
    ctx.fillStyle = gradient; ctx.fillRect(0, 0, width, height);
    ctx.fillStyle = '#4b302f'; ctx.beginPath(); ctx.moveTo(0, height * 0.58); ctx.lineTo(width * 0.2, height * 0.43); ctx.lineTo(width * 0.48, height * 0.54); ctx.lineTo(width * 0.7, height * 0.37); ctx.lineTo(width, height * 0.49); ctx.lineTo(width, height); ctx.lineTo(0, height); ctx.fill();
    const random = seeded(194);
    for (let i = 0; i < 26; i += 1) { const x = random() * width; const y = height * (0.59 + random() * 0.33); const r = 2 + random() * 11; ctx.fillStyle = random() > 0.5 ? '#633b32' : '#9a5e46'; ctx.beginPath(); ctx.ellipse(x, y, r * 1.5, r, random(), 0, Math.PI * 2); ctx.fill(); }
    ctx.fillStyle = '#2d2021'; ctx.fillRect(width * 0.34, height * 0.84, width * 0.32, height * 0.16);
  }

  update(state) {
    const map = mapToWorld(state.pose.x, state.pose.y);
    const ground = terrainHeight(map.x, map.z);
    const heading = state.pose.heading * Math.PI / 180;
    const forward = new THREE.Vector3(Math.cos(heading), 0, Math.sin(heading));
    const cameraName = state.camera;
    this.rover.position.set(map.x, ground, map.z);
    this.rover.rotation.y = Math.PI / 2 - heading;
    this.rover.visible = cameraName === 'arm' || cameraName === 'overhead';
    const bob = Math.sin(state.elapsed * 8) * Math.min(0.035, Math.abs(state.drive.speed) * 0.06);
    const eye = new THREE.Vector3(map.x, ground + 2.04 + bob, map.z);
    let target = eye.clone().add(forward.clone().multiplyScalar(16));
    target.y = ground + 1.62;
    if (cameraName === 'rear') {
      eye.addScaledVector(forward, -0.18); target = eye.clone().addScaledVector(forward, -16); target.y = ground + 1.55;
    } else if (cameraName === 'geo') {
      eye.addScaledVector(forward, -4.2);
      eye.y = ground + 2.35;
      target = this.outcrop.position.clone().add(new THREE.Vector3(0, 1.3, 0));
    } else if (cameraName === 'arm') {
      eye.set(map.x + 4.1, ground + 5.2, map.z + 4.1); target = this.rover.position.clone().add(new THREE.Vector3(0, 0.9, -0.2));
    } else if (cameraName === 'overhead') {
      eye.set(map.x, ground + 14, map.z + 0.1); target = this.rover.position.clone();
    }
    this.currentPosition.lerp(eye, 0.22);
    this.currentTarget.lerp(target, 0.25);
    this.camera.position.copy(this.currentPosition);
    this.camera.lookAt(this.currentTarget);
    if (this.fallback) this._fallback(state);
  }

  render() {
    if (!this.fallback && this.renderer) this.renderer.render(this.scene, this.camera);
  }
}

export { mapToWorld, terrainHeight, MAP_WORLD_SIZE };
