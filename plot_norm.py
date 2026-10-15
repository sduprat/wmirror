import ctypes
import math
import subprocess
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


ROOT = Path(__file__).resolve().parent
LIB_PATH = ROOT / 'build' / 'wmirror_logic.so'
TARGET_AZ = 210.0
TARGET_EL = 47.0
LATITUDE = 43.6
LONGITUDE = 1.8


def build_native_lib():
    LIB_PATH.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        'gcc',
        '-shared',
        '-fPIC',
        '-O2',
        '-I', str(ROOT),
        '-o', str(LIB_PATH),
        str(ROOT / 'wmirror_logic.c'),
        '-lm',
    ]
    subprocess.run(cmd, check=True, cwd=str(ROOT))


def load_lib():
    if not LIB_PATH.exists():
        build_native_lib()
    lib = ctypes.CDLL(str(LIB_PATH))

    lib.horz2cart.argtypes = [
        ctypes.c_double,
        ctypes.c_double,
        ctypes.POINTER(ctypes.c_double * 3),
    ]
    lib.horz2cart.restype = None

    lib.compute_norm_from_angles.argtypes = [
        ctypes.c_double,
        ctypes.c_double,
        ctypes.c_double,
        ctypes.c_double,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
    ]
    lib.compute_norm_from_angles.restype = None
    return lib


LIB = load_lib()


def horz2cart(az_deg, el_deg):
    out = (ctypes.c_double * 3)(0.0, 0.0, 0.0)
    LIB.horz2cart(ctypes.c_double(float(az_deg)), ctypes.c_double(float(el_deg)), out)
    return np.array([out[0], out[1], out[2]], dtype=float)


def normal_from_angles(sun_az, sun_el, target_az, target_el):
    normal_az = ctypes.c_double()
    normal_el = ctypes.c_double()
    LIB.compute_norm_from_angles(
        ctypes.c_double(float(sun_az)),
        ctypes.c_double(float(sun_el)),
        ctypes.c_double(float(target_az)),
        ctypes.c_double(float(target_el)),
        ctypes.byref(normal_az),
        ctypes.byref(normal_el),
    )
    return horz2cart(normal_az.value, normal_el.value)


def angle_between(u, v):
    u = np.asarray(u, dtype=float)
    v = np.asarray(v, dtype=float)
    nu = np.linalg.norm(u)
    nv = np.linalg.norm(v)
    if nu == 0.0 or nv == 0.0:
        return 0.0
    cos_theta = float(np.dot(u, v) / (nu * nv))
    cos_theta = min(1.0, max(-1.0, cos_theta))
    return math.degrees(math.acos(cos_theta))


def representative_sun_path():
    """Generate a representative hourly sun course for a day around the project's latitude/longitude."""
    lat_rad = math.radians(LATITUDE)
    day_of_year = 287  # 14 Oct approx
    points = []

    for hour in range(6, 19):
        solar_hour_angle = math.radians((hour - 12) * 15.0)
        delta = math.radians(23.45 * math.sin(math.radians((360.0 * (day_of_year - 81) / 365.0))))
        altitude = math.asin(
            math.sin(lat_rad) * math.sin(delta)
            + math.cos(lat_rad) * math.cos(delta) * math.cos(solar_hour_angle)
        )
        azimuth = math.atan2(
            math.sin(solar_hour_angle),
            math.cos(solar_hour_angle) * math.sin(lat_rad) - math.tan(delta) * math.cos(lat_rad),
        )

        az_deg = math.degrees(azimuth)
        if az_deg < 0:
            az_deg += 360.0

        points.append((hour, az_deg, math.degrees(altitude)))

    return points


sun_path = representative_sun_path()
fixed_target = horz2cart(TARGET_AZ, TARGET_EL)
rows = []

for hour, sun_az, sun_el in sun_path:
    sun = horz2cart(sun_az, sun_el)
    target = horz2cart(TARGET_AZ, TARGET_EL)
    norm = normal_from_angles(sun_az, sun_el, TARGET_AZ, TARGET_EL)
    dist_to_sun = angle_between(sun, norm)
    dist_to_target = angle_between(target, norm)
    rows.append({
        'hour': hour,
        'sun_az': sun_az,
        'sun_el': sun_el,
        'target_az': TARGET_AZ,
        'target_el': TARGET_EL,
        'normal_az': math.degrees(math.atan2(norm[1], norm[0])),
        'normal_el': math.degrees(math.asin(norm[2])),
        'dist_sun_norm': dist_to_sun,
        'dist_target_norm': dist_to_target,
        'delta_dist': abs(dist_to_sun - dist_to_target),
    })

print('Validation de la médiane : distance angulaire au soleil et à la cible')
print('--------------------------------------------------------------------')
print(f"{'heure':>5} | {'sun_az':>8} | {'sun_el':>8} | {'normal_az':>10} | {'normal_el':>10} | {'dist_sun_norm':>14} | {'dist_target_norm':>16} | {'delta_dist':>11}")
for row in rows:
    print(
        f"{row['hour']:>5} | "
        f"{row['sun_az']:>8.2f} | "
        f"{row['sun_el']:>8.2f} | "
        f"{row['normal_az']:>10.2f} | "
        f"{row['normal_el']:>10.2f} | "
        f"{row['dist_sun_norm']:>14.6f} | "
        f"{row['dist_target_norm']:>16.6f} | "
        f"{row['delta_dist']:>11.6e}"
    )
print('--------------------------------------------------------------------')

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# unit sphere
u = np.linspace(0, 2*np.pi, 40)
v = np.linspace(-np.pi/2, np.pi/2, 20)
X = np.outer(np.cos(u), np.cos(v))
Y = np.outer(np.sin(u), np.cos(v))
Z = np.outer(np.ones_like(u), np.sin(v))
ax.plot_surface(X, Y, Z, color='lightgray', alpha=0.4, linewidth=0, rstride=1, cstride=1)

# fixed target vector
ax.quiver(0, 0, 0, fixed_target[0], fixed_target[1], fixed_target[2], color='black', linewidth=2, label='cible fixe')

for hour, sun_az, sun_el in sun_path:
    sun = horz2cart(sun_az, sun_el)
    norm = normal_from_angles(sun_az, sun_el, TARGET_AZ, TARGET_EL)

    ax.quiver(0, 0, 0, sun[0], sun[1], sun[2], color='red', linewidth=1.2, alpha=0.9)
    ax.quiver(0, 0, 0, norm[0], norm[1], norm[2], color='blue', linewidth=1.2, alpha=0.8)
    ax.text(sun[0], sun[1], sun[2], f'{hour}h', fontsize=8, color='red')

ax.set_xlim(-1.2, 1.2)
ax.set_ylim(-1.2, 1.2)
ax.set_zlim(-1.2, 1.2)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Soleil (rouge), cible fixe (noir), normale (bleu)')
ax.legend(loc='upper right', fontsize='small')

plt.tight_layout()
plt.savefig(ROOT / 'plot_norm.png', dpi=180)
print(f'saved {ROOT / "plot_norm.png"}')
