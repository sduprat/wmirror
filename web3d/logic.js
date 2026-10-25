(function () {
  const normalizeAngle360 = function (angle) {
    let a = angle;
    while (a >= 360.0) a -= 360.0;
    while (a < 0.0) a += 360.0;
    return a;
  };

  const clamp = function (value, minValue, maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
  };

  const horz2cart = function (azimuthDeg, elevationDeg) {
    const az = (azimuthDeg * Math.PI) / 180.0;
    const el = (elevationDeg * Math.PI) / 180.0;
    const cosEl = Math.cos(el);
    return {
      x: cosEl * Math.cos(az),
      y: cosEl * Math.sin(az),
      z: Math.sin(el),
    };
  };

  const cartesianToSpherical = function (v) {
    const { x, y, z } = v;
    const r = Math.sqrt(x * x + y * y + z * z);
    const eps = 1e-12;

    if (Math.abs(r) <= eps) {
      return { azimuthDeg: 0.0, elevationDeg: 0.0 };
    }

    const az = normalizeAngle360((Math.atan2(y, x) * 180.0) / Math.PI);
    const el = (Math.asin(z / r) * 180.0) / Math.PI;

    return { azimuthDeg: az, elevationDeg: el };
  };

  const calculerVecteurNormal = function (incident, reflected) {
    const avg = {
      x: (incident.x + reflected.x) / 2.0,
      y: (incident.y + reflected.y) / 2.0,
      z: (incident.z + reflected.z) / 2.0,
    };

    const length = Math.hypot(avg.x, avg.y, avg.z);
    const eps = 1e-12;

    if (Math.abs(length) <= eps) {
      return { x: 0.0, y: 0.0, z: 0.0 };
    }

    return {
      x: avg.x / length,
      y: avg.y / length,
      z: avg.z / length,
    };
  };

  const computeNormFromAngles = function (sunAzimuthDeg, sunAltitudeDeg, targetAzimuthDeg, targetAltitudeDeg) {
    const sun = horz2cart(sunAzimuthDeg, sunAltitudeDeg);
    const target = horz2cart(targetAzimuthDeg, targetAltitudeDeg);
    const normal = calculerVecteurNormal(sun, target);
    return cartesianToSpherical(normal);
  };

  const roomXYToAzEl = function (x, y, z) {
    const eps = 1e-9;

    if (Math.abs(x) <= eps && Math.abs(y) <= eps) {
      return { azimuthDeg: 0.0, elevationDeg: 90.0 };
    }

    const az = Math.atan2(y, x);
    const rXY = Math.hypot(x, y);
    const el = Math.atan2(z, rXY);

    return {
      azimuthDeg: normalizeAngle360((az * 180.0) / Math.PI),
      elevationDeg: (el * 180.0) / Math.PI,
    };
  };

  const servoAnglesForNormal = function (normalAzimuthDeg, normalAltitudeDeg) {
    const h = 270.0 - normalAzimuthDeg;
    const v = 90.0 - normalAltitudeDeg;
    return {
      h,
      v,
    };
  };

  const clampServoAngles = function (servo) {
    return {
      h: clamp(servo.h, 0.0, 180.0),
      v: clamp(servo.v, 0.0, 90.0),
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

  window.wmirrorLogic = {
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
})();
