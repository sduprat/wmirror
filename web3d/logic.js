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

  const LAVAUR_LATITUDE = 43.6;
  const LAVAUR_LONGITUDE = 1.8;

  const computeSunPosition = function (date = new Date(), lat = LAVAUR_LATITUDE, lon = LAVAUR_LONGITUDE) {
    const timeMs = date.getTime();
    const jd = timeMs / 86400000.0 + 2440587.5;
    const dDays = jd - 2451545.0; // Jours depuis J2000.0

    // Anomalie moyenne du soleil
    const gDeg = 357.529 + 0.98560028 * dDays;
    const gRad = (gDeg % 360.0) * (Math.PI / 180.0);

    // Longitude moyenne du soleil
    const qDeg = 280.459 + 0.98564736 * dDays;
    const qRad = (qDeg % 360.0) * (Math.PI / 180.0);

    // Longitude écliptique apparente
    const lDeg = qDeg + 1.915 * Math.sin(gRad) + 0.020 * Math.sin(2 * gRad);
    const lRad = (lDeg % 360.0) * (Math.PI / 180.0);

    // Obliquité de l'écliptique
    const eDeg = 23.439 - 0.00000036 * dDays;
    const eRad = eDeg * (Math.PI / 180.0);

    // Déclinaison et ascension droite
    const sinD = Math.sin(eRad) * Math.sin(lRad);
    const cosD = Math.cos(Math.asin(sinD));
    const dRad = Math.asin(sinD);
    const raRad = Math.atan2(Math.cos(eRad) * Math.sin(lRad), Math.cos(lRad));

    // Temps sidéral moyen de Greenwich (GMST) en degrés
    const gmstDeg = (280.46061837 + 360.98564736629 * dDays) % 360.0;

    // Temps sidéral local (LMST)
    const lmstDeg = (gmstDeg + lon) % 360.0;
    const lmstRad = lmstDeg * (Math.PI / 180.0);

    // Angle horaire (Hour Angle) normalisé dans [-PI, PI]
    let ha = lmstRad - raRad;
    ha = ((ha + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;

    const latRad = lat * (Math.PI / 180.0);

    // Altitude / Élévation
    let sinAlt = Math.sin(latRad) * Math.sin(dRad) + Math.cos(latRad) * cosD * Math.cos(ha);
    sinAlt = Math.max(-1.0, Math.min(1.0, sinAlt));
    const altRad = Math.asin(sinAlt);
    const elevationDeg = altRad * (180.0 / Math.PI);

    // Azimut (mesuré depuis le Nord, dans le sens horaire vers l'Est : 0° Nord, 90° Est, 180° Sud, 270° Ouest)
    const yAz = -Math.sin(ha);
    const xAz = Math.tan(dRad) * Math.cos(latRad) - Math.sin(latRad) * Math.cos(ha);
    const azRad = Math.atan2(yAz, xAz);
    let azimuthDeg = (azRad * (180.0 / Math.PI)) % 360.0;
    if (azimuthDeg < 0) {
      azimuthDeg += 360.0;
    }

    return {
      azimuthDeg,
      elevationDeg,
    };
  };

  const wmirrorLogic = {
    ready: readyPromise,
    LAVAUR_LATITUDE,
    LAVAUR_LONGITUDE,
    computeSunPosition,
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
