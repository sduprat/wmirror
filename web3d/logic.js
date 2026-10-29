(function () {
  const importObject = {
    env: {
      sin: Math.sin,
      cos: Math.cos,
      atan2: Math.atan2,
      asin: Math.asin,
    },
  };

  let wasmExports = null;
  let doubleView = null;

  async function loadWasm() {
    let wasmBytes;
    if (typeof window !== 'undefined' && typeof fetch !== 'undefined') {
      const response = await fetch('./wmirror_logic.wasm');
      wasmBytes = await response.arrayBuffer();
    } else if (typeof require !== 'undefined') {
      const fs = require('fs');
      const path = require('path');
      wasmBytes = fs.readFileSync(path.join(__dirname, 'wmirror_logic.wasm'));
    }

    const wasmModule = await WebAssembly.instantiate(wasmBytes, importObject);
    wasmExports = wasmModule.instance.exports;
    doubleView = new Float64Array(wasmExports.memory.buffer);
    return wasmExports;
  }

  const readyPromise = loadWasm();

  function getDoubleView() {
    if (doubleView.buffer !== wasmExports.memory.buffer) {
      doubleView = new Float64Array(wasmExports.memory.buffer);
    }
    return doubleView;
  }

  const normalizeAngle360 = function (angle) {
    return wasmExports.normalize_angle_360(angle);
  };

  const clamp = function (value, minValue, maxValue) {
    return wasmExports.clamp_double(value, minValue, maxValue);
  };

  const horz2cart = function (azimuthDeg, elevationDeg) {
    const view = getDoubleView();
    wasmExports.horz2cart(azimuthDeg, elevationDeg, 0);
    return {
      x: view[0],
      y: view[1],
      z: view[2],
    };
  };

  const cartesianToSpherical = function (v) {
    const view = getDoubleView();
    view[0] = v.x;
    view[1] = v.y;
    view[2] = v.z;
    wasmExports.cartesian_to_spherical(0, 24, 32);
    return {
      azimuthDeg: view[3],
      elevationDeg: view[4],
    };
  };

  const calculerVecteurNormal = function (incident, reflected) {
    const view = getDoubleView();
    view[0] = incident.x;
    view[1] = incident.y;
    view[2] = incident.z;
    view[3] = reflected.x;
    view[4] = reflected.y;
    view[5] = reflected.z;
    wasmExports.calculer_vecteur_normal(0, 24, 48);
    return {
      x: view[6],
      y: view[7],
      z: view[8],
    };
  };

  const computeNormFromAngles = function (sunAzimuthDeg, sunAltitudeDeg, targetAzimuthDeg, targetAltitudeDeg) {
    const view = getDoubleView();
    wasmExports.compute_norm_from_angles(
      sunAzimuthDeg,
      sunAltitudeDeg,
      targetAzimuthDeg,
      targetAltitudeDeg,
      0,
      8
    );
    return {
      azimuthDeg: view[0],
      elevationDeg: view[1],
    };
  };

  const roomXYToAzEl = function (x, y, z) {
    const view = getDoubleView();
    wasmExports.room_xy_to_az_el(x, y, z, 0, 8);
    return {
      azimuthDeg: view[0],
      elevationDeg: view[1],
    };
  };

  const servoAnglesForNormal = function (normalAzimuthDeg, normalAltitudeDeg) {
    const h = 270.0 - normalAzimuthDeg;
    const v = 90.0 - normalAltitudeDeg;
    return { h, v };
  };

  const clampServoAngles = function (servo) {
    return {
      h: wasmExports.normalize_servo_h(servo.h),
      v: wasmExports.normalize_servo_v(servo.v),
    };
  };

  const isReachable = function (normalAzimuthDeg, normalAltitudeDeg, cfg) {
    const raw = servoAnglesForNormal(normalAzimuthDeg, normalAltitudeDeg);
    const actual = clampServoAngles(raw);
    const reachable =
      raw.h >= cfg.servoHMin &&
      raw.h <= cfg.servoHMax &&
      raw.v >= cfg.servoVMin &&
      raw.v <= cfg.servoVMax;

    return { ...actual, reachable, raw };
  };

  const wmirrorLogic = {
    ready: readyPromise,
    normalizeAngle360,
    clamp,
    horz2cart,
    cartesianToSpherical,
    calculerVecteurNormal,
    computeNormFromAngles,
    roomXYToAzEl,
    servoAnglesForNormal,
    clampServoAngles,
    isReachable,
  };

  if (typeof window !== 'undefined') {
    window.wmirrorLogic = wmirrorLogic;
  }
  if (typeof module !== 'undefined' && module.exports) {
    module.exports = wmirrorLogic;
  }
})();
