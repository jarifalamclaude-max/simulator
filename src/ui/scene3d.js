// Three.js scene: pool with tiled floor and caustics, water surface, fog,
// marine snow, the vehicle (STL or sample parts), thruster force arrows,
// thruster wash bubbles, CG/CB markers, target-attitude ghost, cameras and the
// preview of thrusters detected in an uploaded design.
//
// Frames: NED world / FRD body are mapped into three.js (y up) by
//   three = (east, -down, -north)  i.e.  (x3, y3, z3) = (y, -z, -x)

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { qToEuler } from '../core/math.js';
import { floorProfile } from '../physics/environment.js';

export const toThree = (v) => new THREE.Vector3(v[1], -v[2], -v[0]);
export const fromThree = (v) => [-v.z, v.x, -v.y];
export const quatToThree = (q) => new THREE.Quaternion(q[2], -q[3], -q[1], q[0]);

const WATER_FOG = new THREE.Color(0x0b4a5c);
const SKY = new THREE.Color(0x9fc8d9);

export class Scene3D {
  constructor(container) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.05;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.append(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.scene.fog = new THREE.FogExp2(WATER_FOG, 0.045);
    this.scene.background = WATER_FOG.clone();
    this.camera = new THREE.PerspectiveCamera(62, 1, 0.03, 400);
    this.camera.position.set(2, -0.6, 2.5);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.maxDistance = 30;
    this.controls.enabled = false;
    this.cameraMode = 'chase';
    this.camTilt = 0; // camera mount tilt (rad, up positive), set from the tilt servo by the app
    this.camPan = 0; // camera mount pan (rad, right positive)
    this.time = 0;
    this.uniforms = { uTime: { value: 0 } };
    this.showThrustArrows = true;
    this.showMarkers = true;
    this.showGhost = true;
    this.lightsLevel = 0;
    this.buildLights();
    this.buildParticles();
    this.vehicle = new THREE.Group();
    this.scene.add(this.vehicle);
    this.ghost = null;
    this.pickHandler = null;
    this.groundPickHandler = null;
    this.raycaster = new THREE.Raycaster();
    this.renderer.domElement.addEventListener('pointerdown', (e) => this.onPointer(e));
    this.resize();
    window.addEventListener('resize', () => this.resize());
    this.chasePos = new THREE.Vector3();
    this.chaseInit = false;
  }

  resize() {
    const w = this.container.clientWidth || window.innerWidth, h = this.container.clientHeight || window.innerHeight;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / Math.max(1, h);
    // shift the projection centre into the part of the screen not covered by
    // the setup panel, top bar and graph dock
    const i = this.insets || { left: 0, right: 0, top: 0, bottom: 0 };
    const dx = (i.left - i.right) / 2, dy = (i.top - i.bottom) / 2;
    if (Math.abs(dx) > 0.5 || Math.abs(dy) > 0.5) this.camera.setViewOffset(w, h, -dx, -dy, w, h);
    else this.camera.clearViewOffset();
    this.camera.updateProjectionMatrix();
  }
  setInsets(i) { this.insets = { left: 0, right: 0, top: 0, bottom: 0, ...i }; this.resize(); }

  buildLights() {
    this.hemi = new THREE.HemisphereLight(0xbfe8f5, 0x0a2a33, 0.9);
    this.scene.add(this.hemi);
    this.sun = new THREE.DirectionalLight(0xffffff, 1.6);
    this.sun.position.set(3, 12, 4);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(1024, 1024);
    const sc = this.sun.shadow.camera;
    sc.left = -2.5; sc.right = 2.5; sc.top = 2.5; sc.bottom = -2.5; sc.near = 0.5; sc.far = 30;
    this.sun.shadow.bias = -0.0005;
    this.sun.shadow.normalBias = 0.025;
    this.scene.add(this.sun, this.sun.target);
  }

  buildPool(pool) {
    if (this.poolGroup) { this.scene.remove(this.poolGroup); this.poolGroup.traverse((o) => { o.geometry?.dispose(); o.material?.dispose?.(); }); }
    const g = new THREE.Group();
    const prof = floorProfile(pool);
    const L = prof.L, W = prof.W, D = prof.maxDepth;
    const F = Math.max(0, Number.isFinite(+pool.freeboard) ? +pool.freeboard : 0.3);
    const natural = pool.style === 'natural';
    this.pool = { L, W, D, prof };
    // rows across the pool at the walls and at the slope breaks (north coordinate)
    const xs = [-L / 2, ...prof.breaks.filter((x) => x > -L / 2 + 1e-6 && x < L / 2 - 1e-6), L / 2].sort((a, b) => a - b);
    const P = (east, depthDown, north) => new THREE.Vector3(east, -depthDown, -north); // NED -> three
    const quads = (list) => {
      // list of [a, b, c, d, inwardNormal]: two triangles each, wound to face the inside
      const pos = [], nrm = [];
      for (const [a, b, c, d, n] of list) {
        const face = new THREE.Vector3().subVectors(b, a).cross(new THREE.Vector3().subVectors(c, a));
        const tri = face.dot(n) >= 0 ? [a, b, c, a, c, d] : [a, c, b, a, d, c];
        const nn = n.clone().normalize();
        for (const v of tri) { pos.push(v.x, v.y, v.z); nrm.push(nn.x, nn.y, nn.z); }
      }
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
      geo.setAttribute('normal', new THREE.Float32BufferAttribute(nrm, 3));
      return geo;
    };
    // floor: standard material + procedural tiles (or a natural bottom), lane lines and caustics
    const floorMat = new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.85, metalness: 0 });
    floorMat.onBeforeCompile = (sh) => {
      sh.uniforms.uTime = this.uniforms.uTime;
      sh.uniforms.uLane = { value: W / 6 };
      sh.vertexShader = sh.vertexShader.replace('#include <common>', '#include <common>\nvarying vec3 vWPos;').replace('#include <worldpos_vertex>', '#include <worldpos_vertex>\nvWPos = (modelMatrix * vec4(transformed, 1.0)).xyz;');
      sh.fragmentShader = sh.fragmentShader
        .replace('#include <common>', `#include <common>
          varying vec3 vWPos; uniform float uTime; uniform float uLane;
          // anti-aliased grout lines: fade to the mean coverage when tiles get small on screen
          float grid(vec2 tp, float hw){
            vec2 fw = max(fwidth(tp), vec2(1e-4));
            vec2 d = abs(fract(tp) - 0.5);
            vec2 l = smoothstep(0.5 - hw - fw, 0.5 - hw + fw, d);
            float g = max(l.x, l.y);
            float fade = clamp(1.6 - max(fw.x, fw.y) * 3.0, 0.0, 1.0);
            return mix(4.0 * hw, g, fade);
          }
          float wv(vec2 p, vec2 d, float f, float s){ return sin(dot(p, d) * f + uTime * s); }
          float caust(vec2 p){
            float c = wv(p, vec2(0.80, 0.60), 3.1, 1.3) + wv(p, vec2(-0.50, 0.87), 3.7, 1.1)
                    + wv(p, vec2(0.20, -0.98), 2.9, 1.7) + wv(p, vec2(-0.92, -0.38), 4.3, 0.9);
            c = abs(c) * 0.25; return pow(1.0 - c, 7.0);
          }
          float hash(vec2 p){ return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
          float vnoise(vec2 p){ vec2 i = floor(p), f = fract(p); f = f * f * (3.0 - 2.0 * f);
            return mix(mix(hash(i), hash(i + vec2(1.0, 0.0)), f.x), mix(hash(i + vec2(0.0, 1.0)), hash(i + vec2(1.0, 1.0)), f.x), f.y); }`)
        .replace('#include <map_fragment>', natural ? `#include <map_fragment>
          float n = 0.55 * vnoise(vWPos.xz * 0.35) + 0.3 * vnoise(vWPos.xz * 1.7) + 0.15 * vnoise(vWPos.xz * 7.0);
          vec3 tile = mix(vec3(0.42, 0.40, 0.33), vec3(0.74, 0.69, 0.56), n);
          diffuseColor.rgb *= tile;` : `#include <map_fragment>
          vec2 tp = vWPos.xz / 0.125;
          vec3 tile = mix(vec3(0.80, 0.90, 0.94), vec3(0.55, 0.66, 0.70), grid(tp, 0.03));
          float lane = abs(fract(vWPos.x / uLane) - 0.5) * uLane;
          float lw = fwidth(vWPos.x);
          tile = mix(tile, vec3(0.06, 0.12, 0.33), (1.0 - smoothstep(0.1 - lw, 0.1 + lw, lane)) * step(abs(vWPos.z), ${Math.max(0, L / 2 - 2).toFixed(2)}));
          diffuseColor.rgb *= tile;`)
        .replace('#include <emissivemap_fragment>', `#include <emissivemap_fragment>
          totalEmissiveRadiance += vec3(0.55, 0.75, 0.8) * caust(vWPos.xz * 1.6) * 0.55 * exp(-max(0.0, -vWPos.y) * 0.08);`);
    };
    const fl = [];
    for (let i = 0; i < xs.length - 1; i++) {
      const x0 = xs[i], x1 = xs[i + 1], d0 = prof.depthAt(x0), d1 = prof.depthAt(x1);
      const up = new THREE.Vector3(0, x1 - x0, -(d1 - d0)); // three-frame normal of the strip, pointing up
      fl.push([P(-W / 2, d0, x0), P(W / 2, d0, x0), P(W / 2, d1, x1), P(-W / 2, d1, x1), up]);
    }
    const floor = new THREE.Mesh(quads(fl), floorMat);
    floor.receiveShadow = true;
    g.add(floor);
    // walls (inside facing; the part above the water is the freeboard)
    const wallMat = new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.9, side: THREE.FrontSide });
    wallMat.onBeforeCompile = (sh) => {
      sh.vertexShader = sh.vertexShader.replace('#include <common>', '#include <common>\nvarying vec3 vWPos;').replace('#include <worldpos_vertex>', '#include <worldpos_vertex>\nvWPos = (modelMatrix * vec4(transformed, 1.0)).xyz;');
      sh.fragmentShader = sh.fragmentShader.replace('#include <common>', `#include <common>
          varying vec3 vWPos;
          float grid(vec2 tp, float hw){
            vec2 fw = max(fwidth(tp), vec2(1e-4));
            vec2 d = abs(fract(tp) - 0.5);
            vec2 l = smoothstep(0.5 - hw - fw, 0.5 - hw + fw, d);
            float g = max(l.x, l.y);
            float fade = clamp(1.6 - max(fw.x, fw.y) * 3.0, 0.0, 1.0);
            return mix(4.0 * hw, g, fade);
          }`)
        .replace('#include <map_fragment>', `#include <map_fragment>
          vec2 tp = (abs(vWPos.x) > abs(vWPos.z) * ${(W / L).toFixed(5)} ? vWPos.zy : vWPos.xy) / 0.125;
          vec3 tile = mix(vec3(0.70, 0.84, 0.89), vec3(0.47, 0.6, 0.65), grid(tp, 0.035));
          // depth marks every metre below the waterline
          float dm = abs(fract(-vWPos.y + 0.5) - 0.5);
          float mw = fwidth(vWPos.y);
          tile = mix(tile, vec3(0.06, 0.13, 0.32), (1.0 - smoothstep(0.025 - mw, 0.025 + mw, dm)) * step(vWPos.y, -0.5));
          // coping above the water
          tile = mix(tile, vec3(0.86, 0.84, 0.79), step(0.0, vWPos.y));
          diffuseColor.rgb *= tile;`);
    };
    const wl = [];
    for (let i = 0; i < xs.length - 1; i++) {
      const x0 = xs[i], x1 = xs[i + 1], d0 = prof.depthAt(x0), d1 = prof.depthAt(x1);
      for (const e of [W / 2, -W / 2]) wl.push([P(e, -F, x0), P(e, -F, x1), P(e, d1, x1), P(e, d0, x0), new THREE.Vector3(-Math.sign(e), 0, 0)]);
    }
    const dN = prof.depthAt(L / 2), dS = prof.depthAt(-L / 2);
    wl.push([P(-W / 2, -F, L / 2), P(W / 2, -F, L / 2), P(W / 2, dN, L / 2), P(-W / 2, dN, L / 2), new THREE.Vector3(0, 0, 1)]); // north wall faces south (+z in three)
    wl.push([P(-W / 2, -F, -L / 2), P(W / 2, -F, -L / 2), P(W / 2, dS, -L / 2), P(-W / 2, dS, -L / 2), new THREE.Vector3(0, 0, -1)]);
    const walls = new THREE.Mesh(quads(wl), wallMat);
    walls.receiveShadow = true;
    walls.visible = !natural;
    g.add(walls);
    // water surface (seen from below and above)
    const surfMat = new THREE.ShaderMaterial({
      transparent: true, side: THREE.DoubleSide, depthWrite: false, fog: true,
      uniforms: THREE.UniformsUtils.merge([THREE.UniformsLib.fog, { uTime: { value: 0 } }]),
      vertexShader: `#include <fog_pars_vertex>
        varying vec3 vW; void main(){ vec4 w = modelMatrix * vec4(position,1.0); vW = w.xyz; vec4 mvPosition = viewMatrix * w; gl_Position = projectionMatrix * mvPosition;
        #include <fog_vertex>
        }`,
      fragmentShader: `#include <common>
        #include <fog_pars_fragment>
        uniform float uTime; varying vec3 vW;
        void main(){
          float r = sin(vW.x * 2.1 + uTime * 1.3) * sin(vW.z * 1.7 - uTime * 1.1) + 0.5 * sin((vW.x + vW.z) * 3.3 + uTime * 2.0);
          vec3 below = mix(vec3(0.55, 0.85, 0.92), vec3(0.9, 0.98, 1.0), smoothstep(0.2, 1.2, r));
          vec3 above = mix(vec3(0.09, 0.38, 0.47), vec3(0.3, 0.6, 0.68), smoothstep(-0.5, 1.2, r));
          bool fromBelow = cameraPosition.y < vW.y;
          gl_FragColor = vec4(fromBelow ? below : above, fromBelow ? 0.78 : 0.62);
          #include <fog_fragment>
        }`,
    });
    this.surfaceMat = surfMat;
    const surf = new THREE.Mesh(new THREE.PlaneGeometry(W, L), surfMat);
    surf.rotation.x = -Math.PI / 2;
    g.add(surf);
    this.poolGroup = g;
    this.scene.add(g);
  }

  buildParticles() {
    const n = 2200, R = 8;
    const pos = new Float32Array(n * 3);
    for (let i = 0; i < n * 3; i++) pos[i] = (Math.random() - 0.5) * 2 * R;
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    const dot = dotTexture();
    const mat = new THREE.PointsMaterial({ color: 0xcfe9ef, size: 0.02, sizeAttenuation: true, transparent: true, opacity: 0.5, depthWrite: false, map: dot, alphaTest: 0.02 });
    this.snow = new THREE.Points(geo, mat);
    this.snow.frustumCulled = false;
    this.snowR = R;
    this.scene.add(this.snow);
    // thruster wash bubbles
    const bn = 900;
    const bg = new THREE.BufferGeometry();
    this.bubblePos = new Float32Array(bn * 3).fill(1e5);
    this.bubbleVel = new Float32Array(bn * 3);
    this.bubbleLife = new Float32Array(bn);
    bg.setAttribute('position', new THREE.BufferAttribute(this.bubblePos, 3));
    this.bubbles = new THREE.Points(bg, new THREE.PointsMaterial({ color: 0xe8fbff, size: 0.014, transparent: true, opacity: 0.75, depthWrite: false, map: dot, alphaTest: 0.02 }));
    this.bubbles.frustumCulled = false;
    this.bubbleNext = 0;
    this.scene.add(this.bubbles);
  }

  // CFD flow overlay (FlowView from flow3d.js): streamlines in the vehicle
  // frame and the hull coloured by pressure coefficient
  setFlowOverlay(view, { streamlines = true, pressure = true } = {}) {
    this.clearFlowOverlay();
    if (!view || !this.body) return;
    this.flowGroup = new THREE.Group();
    if (streamlines) this.flowGroup.add(view.streamlines());
    this.body.add(this.flowGroup);
    if (pressure && this.meshes?.length) {
      this.flowSaved = this.meshes.map((m) => m.material);
      for (const m of this.meshes) {
        m.geometry.setAttribute('color', new THREE.BufferAttribute(view.hullColors(m.geometry.attributes.position), 3));
        m.material = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.6, metalness: 0, flatShading: m.material.flatShading });
      }
    }
  }
  clearFlowOverlay() {
    if (this.flowGroup) {
      this.flowGroup.parent?.remove(this.flowGroup);
      this.flowGroup.traverse((o) => { o.geometry?.dispose(); o.material?.dispose?.(); });
      this.flowGroup = null;
    }
    if (this.flowSaved && this.meshes) this.meshes.forEach((m, i) => { m.material.dispose(); m.material = this.flowSaved[i]; m.geometry.deleteAttribute('color'); });
    this.flowSaved = null;
  }

  // Geometry: parts [{positions (FRD m, vehicle frame), color, name}]
  setVehicleGeometry(parts, color) {
    this.clearFlowOverlay();
    if (this.body) { this.vehicle.remove(this.body); this.body.traverse((o) => { o.geometry?.dispose(); o.material?.dispose?.(); }); }
    this.meshes = [];
    this.body = new THREE.Group();
    this.vehicle.add(this.body);
    this.hull = new THREE.Group();
    this.body.add(this.hull);
    for (const p of parts) {
      const arr = new Float32Array(p.positions.length);
      for (let i = 0; i < p.positions.length; i += 3) { arr[i] = p.positions[i + 1]; arr[i + 1] = -p.positions[i + 2]; arr[i + 2] = -p.positions[i]; }
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.BufferAttribute(arr, 3));
      geo.computeVertexNormals();
      const mat = new THREE.MeshStandardMaterial({ color: new THREE.Color(p.color || color || '#dfe6ea'), roughness: 0.55, metalness: 0.15, flatShading: arr.length / 9 < 20000 });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.castShadow = true;
      mesh.userData.part = p.name;
      this.hull.add(mesh);
      this.meshes.push(mesh);
    }
    this.overlay = null;
    this.chaseInit = false;
  }

  // Overlays that follow the configuration: CG offset, thruster arrows/labels,
  // CG/CB markers, axes, target ghost, headlights.
  setOverlays({ thrusters, cg, cb, bbox }) {
    if (!this.body) return;
    if (this.overlay) { this.body.remove(this.overlay); this.overlay.traverse((o) => { o.geometry?.dispose(); o.material?.map?.dispose?.(); o.material?.dispose?.(); }); }
    if (this.ghost) { this.scene.remove(this.ghost); this.ghost.geometry.dispose(); }
    if (this.spot) { this.vehicle.remove(this.spot, this.spot.target); }
    const ov = new THREE.Group();
    this.overlay = ov;
    this.body.add(ov);
    this.cgThree = toThree(cg);
    this.body.position.copy(this.cgThree).multiplyScalar(-1);
    this.arrows = [];
    this.thrusterDefs = thrusters;
    thrusters.forEach((t, i) => {
      const a = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), toThree(t.pos), 0.1, 0xffb547, 0.05, 0.035);
      a.line.material.depthTest = false; a.cone.material.depthTest = false;
      a.renderOrder = 10;
      ov.add(a);
      const label = makeLabel(String(i + 1), t.enabled === false ? '#6f8a97' : '#e3eef3');
      label.position.copy(toThree(t.pos)).add(new THREE.Vector3(0, 0.045, 0));
      ov.add(label);
      this.arrows.push({ arrow: a, label });
    });
    this.markers = new THREE.Group();
    const cgM = new THREE.Mesh(new THREE.SphereGeometry(0.018, 16, 12), new THREE.MeshBasicMaterial({ color: 0xffd23f, depthTest: false }));
    cgM.position.copy(toThree(cg));
    const cbM = new THREE.Mesh(new THREE.SphereGeometry(0.018, 16, 12), new THREE.MeshBasicMaterial({ color: 0x4f8dff, depthTest: false }));
    cbM.position.copy(toThree(cb));
    cgM.renderOrder = cbM.renderOrder = 11;
    this.markers.add(cgM, cbM, makeLabel('CG', '#ffd23f', cgM.position.clone().add(new THREE.Vector3(0.05, 0.03, 0))), makeLabel('CB', '#8fb3ff', cbM.position.clone().add(new THREE.Vector3(0.05, 0.03, 0))));
    const axes = new THREE.Group();
    const L = Math.max(...bbox.size) * 0.45;
    axes.add(new THREE.ArrowHelper(toThree([1, 0, 0]), toThree(cg), L, 0xff5a5a, 0.04, 0.025));
    axes.add(new THREE.ArrowHelper(toThree([0, 1, 0]), toThree(cg), L * 0.6, 0x5ad17a, 0.04, 0.025));
    axes.add(new THREE.ArrowHelper(toThree([0, 0, 1]), toThree(cg), L * 0.6, 0x5a8dff, 0.04, 0.025));
    axes.traverse((o) => { if (o.material) { o.material.depthTest = false; o.renderOrder = 9; } });
    this.markers.add(axes);
    ov.add(this.markers);
    this.markers.visible = this.showMarkers;
    const s = bbox.size;
    this.ghost = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(s[1], s[2], s[0])), new THREE.LineBasicMaterial({ color: 0x5cc8e0, transparent: true, opacity: 0.55 }));
    this.scene.add(this.ghost);
    this.ghostOffset = toThree(bbox.center).sub(this.cgThree);
    this.spot = new THREE.SpotLight(0xfff4e0, 0, 9, 0.5, 0.45, 1.2);
    this.spot.position.copy(toThree([bbox.max[0], bbox.center[1], bbox.center[2]]).sub(this.cgThree));
    this.spot.target.position.copy(this.spot.position.clone().add(toThree([2, 0, 0.2])));
    this.vehicle.add(this.spot, this.spot.target);
    this.bodySize = Math.max(...s);
    this.bboxMaxX = bbox.max[0];
    this.bboxCenter = bbox.center;
    this.drawPreview();
  }

  // Detected thrusters waiting for Accept / Discard: a fixed arrow along the
  // push direction and a ring of the duct diameter at each one (cyan = sure,
  // amber = check). list: [{pos, axis, od, confidence}] or null to clear.
  setThrusterPreview(list) {
    this.previewDefs = list && list.length ? list : null;
    this.drawPreview();
  }
  drawPreview() {
    if (this.preview) {
      this.preview.parent?.remove(this.preview);
      this.preview.traverse((o) => { o.geometry?.dispose(); o.material?.dispose?.(); });
      this.preview = null;
    }
    if (!this.previewDefs || !this.overlay) return;
    const g = new THREE.Group();
    const L = Math.min(0.25, Math.max(0.06, (this.bodySize || 0.5) * 0.22));
    for (const t of this.previewDefs) {
      const color = t.confidence >= 0.6 ? 0x5cc8e0 : 0xfab219;
      const dir = toThree(t.axis).normalize(), at = toThree(t.pos);
      const a = new THREE.ArrowHelper(dir, at, L, color, L * 0.3, L * 0.18);
      const r = Math.max(0.015, (t.od || 0.1) / 2);
      const pts = Array.from({ length: 41 }, (_, k) => new THREE.Vector3(Math.cos((k / 40) * 2 * Math.PI) * r, Math.sin((k / 40) * 2 * Math.PI) * r, 0));
      const ring = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color }));
      ring.position.copy(at);
      ring.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), dir);
      for (const o of [a.line, a.cone, ring]) { o.material.depthTest = false; o.renderOrder = 13; }
      g.add(a, ring);
    }
    this.overlay.add(g);
    this.preview = g;
  }

  setPickHandler(fn) { this.pickHandler = fn; this.renderer.domElement.style.cursor = fn ? 'crosshair' : ''; }
  // click on the water at the vehicle's depth (top view): fn([north, east, down]) - used for GUIDED "go to"
  setGroundPickHandler(fn) { this.groundPickHandler = fn; if (!this.pickHandler) this.renderer.domElement.style.cursor = fn ? 'crosshair' : ''; }
  onPointer(e) {
    const r = this.renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    if (!this.pickHandler && this.groundPickHandler && this.lastPos) {
      this.raycaster.setFromCamera(ndc, this.camera);
      const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -this.lastPos.y);
      const at = new THREE.Vector3();
      if (this.raycaster.ray.intersectPlane(plane, at)) this.groundPickHandler(fromThree(at), e);
      return;
    }
    if (!this.pickHandler || !this.meshes?.length) return;
    this.raycaster.setFromCamera(ndc, this.camera);
    const hit = this.raycaster.intersectObjects(this.meshes, false)[0];
    if (!hit) return;
    const local = this.body.worldToLocal(hit.point.clone());
    const nrm = hit.face ? hit.face.normal.clone() : new THREE.Vector3(0, 1, 0);
    this.pickHandler(fromThree(local), fromThree(nrm));
  }

  setCameraMode(m) {
    this.cameraMode = m;
    this.controls.enabled = m === 'orbit' || m === 'free';
    this.chaseInit = false;
    if (m === 'orbit' && this.lastPos) { this.controls.target.copy(this.lastPos); }
  }

  // sim: Simulation, dt: seconds of wall clock
  update(sim, dt) {
    this.time += dt;
    this.uniforms.uTime.value = this.time;
    if (this.surfaceMat) this.surfaceMat.uniforms.uTime.value = this.time;
    const p = sim.plant.position, q = sim.plant.quaternion;
    const pos = toThree(p);
    const qt = quatToThree(q);
    this.vehicle.position.copy(pos);
    this.vehicle.quaternion.copy(qt);
    this.lastPos = pos;
    // thrust arrows
    if (this.arrows) {
      const th = sim.plant.thrusters;
      this.arrows.forEach((a, i) => {
        const t = th[i];
        if (!t) return;
        const def = this.thrusterDefs[i];
        const Tmax = Math.max(1, t.curve.fwdMax);
        const k = t.Teff / Tmax;
        const dir = toThree(def.axis).normalize().multiplyScalar(Math.sign(k) || 1);
        a.arrow.visible = this.showThrustArrows && Math.abs(k) > 0.01;
        a.label.visible = this.showMarkers;
        if (a.arrow.visible) {
          const len = 0.04 + Math.abs(k) * this.bodySize * 0.55;
          a.arrow.setDirection(dir);
          a.arrow.setLength(len, Math.min(0.05, len * 0.35), 0.03);
          a.arrow.setColor(k > 0 ? 0xffb547 : 0x4f8dff);
        }
        // wash bubbles out of the jet side
        if (Math.abs(k) > 0.15 && this.bubbleNext < 1e9) this.emitBubbles(def, k, dt);
      });
    }
    this.updateBubbles(dt);
    // ghost target attitude
    if (this.ghost) {
      const fw = sim.fw;
      const show = this.showGhost && fw.armed && fw.mode !== 19;
      this.ghost.visible = show;
      if (show) {
        const e = fw.att.eulerAngleTarget;
        const qg = quatToThree(eulerQuat(e[0], e[1], e[2]));
        this.ghost.position.copy(pos).add(this.ghostOffset.clone().applyQuaternion(qg));
        this.ghost.quaternion.copy(qg);
      }
    }
    // lights: brightness of the lamps on the lights1 / lights2 servo outputs (SERVOn_FUNCTION 181/182)
    const js = sim.fw.js;
    const lv = js.lightLevel ? Math.max(js.lightLevel(0), js.lightLevel(1)) : js.lights1 / 1000;
    this.spot.intensity = lv * 9;
    // marine snow drifts with the water and wraps around the camera
    const cur = sim.env.cfg.current;
    const drift = toThree([cur[0] + sim.env.gust[0], cur[1] + sim.env.gust[1], cur[2]]).multiplyScalar(dt);
    const sp = this.snow.geometry.attributes.position.array, R = this.snowR;
    const c = this.camera.position;
    for (let i = 0; i < sp.length; i += 3) {
      sp[i] += drift.x; sp[i + 1] += drift.y + Math.sin(this.time * 0.3 + i) * 0.024 * dt; sp[i + 2] += drift.z;
      for (let k = 0; k < 3; k++) {
        const cc = k === 0 ? c.x : k === 1 ? c.y : c.z;
        if (sp[i + k] - cc > R) sp[i + k] -= 2 * R; else if (sp[i + k] - cc < -R) sp[i + k] += 2 * R;
      }
      if (sp[i + 1] > -0.02) sp[i + 1] -= 0.5;
    }
    this.snow.geometry.attributes.position.needsUpdate = true;
    // camera
    this.updateCamera(sim, pos, qt, dt);
    // shadow camera follows the vehicle
    this.sun.position.set(pos.x + 3, 12, pos.z + 4);
    this.sun.target.position.copy(pos);
    // underwater vs above-water look; daylight fades with depth
    // (diffuse attenuation ~ 1 / (1.5 x visibility)), so deep water gets dark
    const under = this.camera.position.y < 0;
    const vis = Math.max(2, sim.env.cfg.visibility ?? 30);
    const att = Math.max(0.06, Math.exp(-Math.max(0, p[2]) / (1.5 * vis)));
    this.hemi.intensity = 0.9 * Math.max(0.2, att);
    this.sun.intensity = 1.6 * att;
    const fogC = under ? (this.fogTmp ||= new THREE.Color()).copy(WATER_FOG).multiplyScalar(0.2 + 0.8 * att) : SKY;
    this.scene.fog.color.copy(fogC);
    this.scene.fog.density = under ? 1.73 / Math.max(2, sim.env.cfg.visibility ?? 30) : 0.004;
    this.scene.background.copy(fogC);
    this.renderer.render(this.scene, this.camera);
  }

  emitBubbles(def, k, dt) {
    const rate = Math.abs(k) * 220 * dt;
    let n = Math.floor(rate + Math.random());
    const jet = toThree(def.axis).normalize().multiplyScalar(-Math.sign(k));
    const origin = this.body.localToWorld(toThree(def.pos));
    const jw = jet.applyQuaternion(this.vehicle.quaternion);
    const bp = this.bubblePos, bv = this.bubbleVel, bl = this.bubbleLife;
    const N = bl.length;
    while (n-- > 0) {
      const i = this.bubbleNext++ % N;
      bp[i * 3] = origin.x + (Math.random() - 0.5) * 0.03; bp[i * 3 + 1] = origin.y + (Math.random() - 0.5) * 0.03; bp[i * 3 + 2] = origin.z + (Math.random() - 0.5) * 0.03;
      const s = 0.6 + Math.random() * 0.8 * Math.abs(k);
      bv[i * 3] = jw.x * s + (Math.random() - 0.5) * 0.15; bv[i * 3 + 1] = jw.y * s + (Math.random() - 0.5) * 0.15; bv[i * 3 + 2] = jw.z * s + (Math.random() - 0.5) * 0.15;
      bl[i] = 0.6 + Math.random() * 0.6;
    }
  }
  updateBubbles(dt) {
    const bp = this.bubblePos, bv = this.bubbleVel, bl = this.bubbleLife;
    const damp = Math.pow(0.94, dt * 60); // 6 % per 1/60 s, whatever the frame rate
    for (let i = 0; i < bl.length; i++) {
      if (bl[i] <= 0) { if (bp[i * 3] < 1e4) bp[i * 3] = 1e5; continue; }
      bl[i] -= dt;
      bv[i * 3] *= damp; bv[i * 3 + 1] = bv[i * 3 + 1] * damp + 0.25 * dt; bv[i * 3 + 2] *= damp;
      bp[i * 3] += bv[i * 3] * dt; bp[i * 3 + 1] += bv[i * 3 + 1] * dt; bp[i * 3 + 2] += bv[i * 3 + 2] * dt;
      if (bp[i * 3 + 1] > -0.01) bl[i] = 0;
    }
    this.bubbles.geometry.attributes.position.needsUpdate = true;
  }

  updateCamera(sim, pos, qt, dt) {
    const cam = this.camera;
    const L = Math.max(0.4, this.bodySize || 0.5);
    if (this.cameraMode === 'orbit') {
      this.controls.target.lerp(pos, 1 - Math.exp(-dt * 8));
      this.controls.update();
      return;
    }
    if (this.cameraMode === 'free') { this.controls.update(); return; }
    const yaw = qToEuler(sim.plant.quaternion)[2];
    const headingQ = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -yaw);
    if (this.cameraMode === 'chase') {
      const off = new THREE.Vector3(0, L * 0.9, L * 3.4).applyQuaternion(headingQ);
      const want = pos.clone().add(off);
      if (!this.chaseInit) { this.chasePos.copy(want); this.chaseInit = true; }
      this.chasePos.lerp(want, 1 - Math.exp(-dt * 3));
      cam.position.copy(this.chasePos);
      cam.up.set(0, 1, 0);
      cam.lookAt(pos.clone().add(new THREE.Vector3(0, L * 0.15, 0)));
    } else if (this.cameraMode === 'top') {
      cam.position.copy(pos).add(new THREE.Vector3(0, L * 7, 0.001));
      cam.up.set(0, 0, -1).applyQuaternion(headingQ);
      cam.lookAt(pos);
    } else if (this.cameraMode === 'fpv') {
      const nose = this.body.localToWorld(toThree([this.bboxMaxX + 0.01, this.bboxCenter?.[1] ?? 0, this.bboxCenter?.[2] ?? 0]));
      cam.position.copy(nose);
      const tilt = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), this.camTilt);
      const look = new THREE.Quaternion().copy(qt).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -this.camPan)).multiply(tilt);
      cam.quaternion.copy(look);
    }
  }

  // AUTO mission waypoints and the GUIDED target, NED metres: [{pos:[n,e,d]}], target [n,e,d] | null
  setWaypoints(list = [], target = null) {
    if (this.wpGroup) { this.scene.remove(this.wpGroup); this.wpGroup.traverse((o) => { o.geometry?.dispose?.(); o.material?.dispose?.(); }); }
    const g = new THREE.Group();
    const mk = (p, color, r) => { const m = new THREE.Mesh(new THREE.SphereGeometry(r, 16, 12), new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.85, depthTest: false })); m.position.copy(toThree(p)); m.renderOrder = 14; return m; };
    const pts = list.filter((w) => Array.isArray(w?.pos)).map((w) => w.pos);
    pts.forEach((p, i) => g.add(mk(p, i === 0 ? 0x7fe0ad : 0x5cc8e0, 0.06)));
    if (pts.length > 1) {
      const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts.map(toThree)), new THREE.LineBasicMaterial({ color: 0x5cc8e0, depthTest: false, transparent: true, opacity: 0.8 }));
      line.renderOrder = 14; g.add(line);
    }
    if (target) g.add(mk(target, 0xffb547, 0.08));
    this.wpGroup = g;
    this.scene.add(g);
  }

  setVisibility({ arrows, markers, ghost }) {
    if (arrows !== undefined) this.showThrustArrows = arrows;
    if (markers !== undefined) { this.showMarkers = markers; if (this.markers) this.markers.visible = markers; }
    if (ghost !== undefined) this.showGhost = ghost;
  }
  setColor(color) { for (const m of this.meshes || []) if (m.userData.part === 'Hull') m.material.color.set(color); }
  setGhostOpacity(o) { if (this.ghost) this.ghost.material.opacity = o; }
}

function eulerQuat(r, p, y) {
  const cr = Math.cos(r / 2), sr = Math.sin(r / 2), cp = Math.cos(p / 2), sp = Math.sin(p / 2), cy = Math.cos(y / 2), sy = Math.sin(y / 2);
  return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy];
}

function makeLabel(text, color = '#e3eef3', pos) {
  const c = document.createElement('canvas');
  c.width = 64; c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = 'rgba(4,14,20,0.78)';
  g.beginPath(); g.arc(32, 32, 26, 0, Math.PI * 2); g.fill();
  g.fillStyle = color; g.font = '600 30px "IBM Plex Mono", monospace'; g.textAlign = 'center'; g.textBaseline = 'middle';
  g.fillText(text, 32, 34);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  s.scale.set(0.045, 0.045, 1);
  s.renderOrder = 12;
  if (pos) s.position.copy(pos);
  return s;
}

// soft round sprite for particles
function dotTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 32;
  const g = c.getContext('2d');
  const gr = g.createRadialGradient(16, 16, 0, 16, 16, 16);
  gr.addColorStop(0, 'rgba(255,255,255,1)');
  gr.addColorStop(0.45, 'rgba(255,255,255,0.75)');
  gr.addColorStop(1, 'rgba(255,255,255,0)');
  g.fillStyle = gr;
  g.fillRect(0, 0, 32, 32);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}
