const {
  computeNormFromAngles,
  horz2cart,
  isReachable,
  roomXYToAzEl,
} = window.wmirrorLogic;

const canvas = document.querySelector('#scene');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

const scene = new THREE.Scene();
scene.background = null;

const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 1000);
camera.position.set(12, 10, 12);
camera.lookAt(0, 0, 0);

const controls = {
  targetX: 0.0,
  targetY: 0.0,
  targetZ: 2.5,
  sunAz: 240.0,
  sunEl: 15.0,
  servoHMin: 0,
  servoHMax: 180,
  servoVMin: 0,
  servoVMax: 90,
};

const roomOrigin = new THREE.Vector3(0, 0.05, 0);

const root = new THREE.Group();
scene.add(root);

let autoRotate = false;

const ambient = new THREE.AmbientLight(0xffffff, 0.9);
scene.add(ambient);

const directional = new THREE.DirectionalLight(0xffffff, 0.6);
directional.position.set(5, 8, 6);
scene.add(directional);

const gridHelper = new THREE.GridHelper(40, 40, 0x3a4d78, 0x25375e);
root.add(gridHelper);

const axes = new THREE.AxesHelper(6);
root.add(axes);

const roomPlane = new THREE.Mesh(
  new THREE.PlaneGeometry(26, 26),
  new THREE.MeshBasicMaterial({
    color: 0x1f2e4a,
    side: THREE.DoubleSide,
    transparent: true,
    opacity: 0.26,
  })
);
roomPlane.rotation.x = -Math.PI / 2;
roomPlane.position.y = 0;
root.add(roomPlane);

const targetPoint = new THREE.Mesh(
  new THREE.SphereGeometry(0.18, 20, 20),
  new THREE.MeshStandardMaterial({ color: 0xf7d779, emissive: 0x4d3a12 })
);
root.add(targetPoint);

function makeVectorMesh(color, length, radius = 0.08) {
  const group = new THREE.Group();
  const body = new THREE.Mesh(
    new THREE.CylinderGeometry(radius, radius, length, 16),
    new THREE.MeshStandardMaterial({
      color,
      emissive: color,
      emissiveIntensity: 0.25,
      transparent: true,
      opacity: 0.95,
    })
  );

  const tip = new THREE.Mesh(
    new THREE.ConeGeometry(radius * 2.8, radius * 5.2, 16),
    new THREE.MeshStandardMaterial({
      color,
      emissive: color,
      emissiveIntensity: 0.25,
      transparent: true,
      opacity: 0.95,
    })
  );

  body.position.y = length / 2;
  tip.position.y = length + radius * 2.6;
  group.add(body, tip);

  return group;
}

const sunVector = makeVectorMesh(0x5ab4ff, 4.5);
const targetVectorMesh = makeVectorMesh(0xf7d779, 4.2);
const normalVectorMesh = makeVectorMesh(0x53d68c, 3.5);
root.add(sunVector, targetVectorMesh, normalVectorMesh);

const sunDir = new THREE.Vector3();
const targetVector = new THREE.Vector3();
const normalVector = new THREE.Vector3();

const reachablePoints = [];
const reachableGroup = new THREE.Group();
root.add(reachableGroup);

const ceilingPoints = [];
const ceilingGroup = new THREE.Group();
root.add(ceilingGroup);

const outputEls = {
  az: document.getElementById('outAz'),
  el: document.getElementById('outEl'),
  h: document.getElementById('outH'),
  v: document.getElementById('outV'),
};

function attachSlider(id, key, formatter = (v) => Number(v).toFixed(1)) {
  const el = document.getElementById(id);
  const val = document.getElementById(`${id}Val`);
  const apply = () => {
    controls[key] = Number(el.value);
    val.textContent = formatter(controls[key]);
    update();
  };
  el.addEventListener('input', apply);
  apply();
}

attachSlider('targetX', 'targetX', (v) => Number(v).toFixed(1));
attachSlider('targetY', 'targetY', (v) => Number(v).toFixed(1));
attachSlider('targetZ', 'targetZ', (v) => Number(v).toFixed(1));
attachSlider('sunAz', 'sunAz', (v) => Number(v).toFixed(0));
attachSlider('sunEl', 'sunEl', (v) => Number(v).toFixed(0));
attachSlider('servoHMin', 'servoHMin', (v) => Number(v).toFixed(0));
attachSlider('servoHMax', 'servoHMax', (v) => Number(v).toFixed(0));
attachSlider('servoVMin', 'servoVMin', (v) => Number(v).toFixed(0));
attachSlider('servoVMax', 'servoVMax', (v) => Number(v).toFixed(0));

function sampleGrid() {
  reachablePoints.length = 0;
  reachableGroup.clear();
  ceilingPoints.length = 0;
  ceilingGroup.clear();

  for (let x = -7.5; x <= 7.5; x += 0.3) {
    for (let y = -7.5; y <= 7.5; y += 0.3) {
      const azEl = roomXYToAzEl(x, y, controls.targetZ);
      const normal = computeNormFromAngles(controls.sunAz, controls.sunEl, azEl.azimuthDeg, azEl.elevationDeg);
      const result = isReachable(normal.azimuthDeg, normal.elevationDeg, controls);

      const floorPoint = new THREE.Mesh(
        new THREE.SphereGeometry(0.05, 8, 8),
        new THREE.MeshStandardMaterial({
          color: 0x4d5868,
          emissive: 0x273041,
          transparent: true,
          opacity: 0.45,
        })
      );
      floorPoint.position.set(x, 0.1, y);
      reachableGroup.add(floorPoint);
      reachablePoints.push(floorPoint);

      const ceilingPoint = new THREE.Mesh(
        new THREE.SphereGeometry(0.05, 8, 8),
        new THREE.MeshStandardMaterial({
          color: result.reachable ? 0xf7d779 : 0xff6b6b,
          emissive: result.reachable ? 0x4d3a12 : 0x4d1212,
        })
      );
      ceilingPoint.position.set(x, 3.2, y);
      ceilingGroup.add(ceilingPoint);
      ceilingPoints.push(ceilingPoint);
    }
  }
}

function updateTargetRepresentation() {
  const target = new THREE.Vector3(controls.targetX, controls.targetZ, controls.targetY);
  targetPoint.position.copy(target);

  const sun = horz2cart(controls.sunAz, controls.sunEl);
  sunDir.set(sun.x, sun.z, sun.y);
  sunDir.normalize();
  sunVector.scale.set(1, 1, 1);
  sunVector.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), sunDir.clone().normalize());
  sunVector.position.set(0, 0, 0);
  sunVector.scale.set(1, 1, 1);

  const azEl = roomXYToAzEl(controls.targetX, controls.targetY, controls.targetZ);
  const targetVec = horz2cart(azEl.azimuthDeg, azEl.elevationDeg);
  targetVector.set(targetVec.x, targetVec.z, targetVec.y).normalize();
  targetVectorMesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), targetVector.clone().normalize());
  targetVectorMesh.position.set(0, 0, 0);

  const normal = computeNormFromAngles(controls.sunAz, controls.sunEl, azEl.azimuthDeg, azEl.elevationDeg);
  const normalVec = horz2cart(normal.azimuthDeg, normal.elevationDeg);
  normalVector.set(normalVec.x, normalVec.z, normalVec.y).normalize();
  normalVectorMesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), normalVector.clone().normalize());
  normalVectorMesh.position.set(0, 0, 0);

  const servo = isReachable(normal.azimuthDeg, normal.elevationDeg, controls);
  outputEls.az.textContent = `${azEl.azimuthDeg.toFixed(1)}°`;
  outputEls.el.textContent = `${azEl.elevationDeg.toFixed(1)}°`;
  outputEls.h.textContent = `${servo.h.toFixed(1)}°`;
  outputEls.v.textContent = `${servo.v.toFixed(1)}°`;
}

function update() {
  updateTargetRepresentation();
  sampleGrid();
}

function resize() {
  const { clientWidth, clientHeight } = canvas.parentElement;
  renderer.setSize(clientWidth, clientHeight, false);
  camera.aspect = clientWidth / clientHeight;
  camera.updateProjectionMatrix();
}

window.addEventListener('resize', resize);
document.getElementById('scanBtn').addEventListener('click', () => {
  sampleGrid();
});

resize();
update();

function updateRotationUI() {
  const btn = document.getElementById('toggleRotateBtn');
  btn.textContent = `Rotation: ${autoRotate ? 'on' : 'off'}`;
}

document.getElementById('toggleRotateBtn').addEventListener('click', () => {
  autoRotate = !autoRotate;
  updateRotationUI();
});

function animate() {
  requestAnimationFrame(animate);
  if (autoRotate) {
    root.rotation.y += 0.006;
  }
  renderer.render(scene, camera);
}

updateRotationUI();
animate();
