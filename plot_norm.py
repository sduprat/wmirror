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


def scan_target_square_around_fixed_sun(sun_az_deg=15.0,
                                      sun_el_deg=15.0,
                                      start_target_az=TARGET_AZ,
                                      start_target_el=TARGET_EL,
                                      half_span_deg=50.0,
                                      step_deg=5.0):
    """Parcours du contour d'un rectangle autour du point de départ : 210/47 → 210/30 → 180/30 → 180/60 → 210/60 → 210/47."""
    rows = []
    path = []

    for el in np.arange(start_target_el, 30.0 - 1e-9, -step_deg):
        path.append((start_target_az, el))
    path.append((start_target_az, 30.0))

    for az in np.arange(start_target_az, 180.0 - 1e-9, -step_deg):
        path.append((az, 30.0))
    path.append((180.0, 30.0))

    for el in np.arange(30.0, 60.0 + 1e-9, step_deg):
        path.append((180.0, el))
    path.append((180.0, 60.0))

    for az in np.arange(180.0, start_target_az + 1e-9, step_deg):
        path.append((az, 60.0))
    path.append((start_target_az, 60.0))

    for el in np.arange(60.0, start_target_el - 1e-9, -step_deg):
        path.append((start_target_az, el))
    path.append((start_target_az, start_target_el))

    for target_az, target_el in path:
        target_az = float(target_az)
        target_el = float(target_el)
        sun = horz2cart(sun_az_deg, sun_el_deg)
        target = horz2cart(target_az, target_el)
        norm = normal_from_angles(sun_az_deg, sun_el_deg, target_az, target_el)
        dist_to_sun = angle_between(sun, norm)
        dist_to_target = angle_between(target, norm)
        rows.append({
            'sun_az': sun_az_deg,
            'sun_el': sun_el_deg,
            'target_az': target_az,
            'target_el': target_el,
            'az_offset': target_az - start_target_az,
            'el_offset': target_el - start_target_el,
            'normal_az': math.degrees(math.atan2(norm[1], norm[0])),
            'normal_el': math.degrees(math.asin(norm[2])),
            'dist_sun_norm': dist_to_sun,
            'dist_target_norm': dist_to_target,
            'delta_dist': abs(dist_to_sun - dist_to_target),
        })
    return rows


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


def plot_fixed_target_validation(rows):
    """Graphique 1 : cible fixe et soleil sur le trajet représentatif."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    u = np.linspace(0, 2*np.pi, 40)
    v = np.linspace(-np.pi/2, np.pi/2, 20)
    X = np.outer(np.cos(u), np.cos(v))
    Y = np.outer(np.sin(u), np.cos(v))
    Z = np.outer(np.ones_like(u), np.sin(v))
    ax.plot_surface(X, Y, Z, color='lightgray', alpha=0.4, linewidth=0, rstride=1, cstride=1)

    fixed_target = horz2cart(TARGET_AZ, TARGET_EL)
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
    fig.savefig(ROOT / 'plot_norm.png', dpi=180)
    print(f'saved {ROOT / "plot_norm.png"}')


def plot_fixed_sun_square_validation(rows, sun_az_deg=15.0, sun_el_deg=15.0):
    """Graphique 2 : vue 3D de la pièce, avec plafond et intersections de chaque vecteur sur le plafond."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # room box and ceiling
    room_x0, room_x1 = 0.0, 12.0
    room_y0, room_y1 = 0.0, 8.0
    room_z0, room_z1 = 0.0, 2.5
    vertices = [
        (room_x0, room_y0, room_z0),
        (room_x1, room_y0, room_z0),
        (room_x1, room_y1, room_z0),
        (room_x0, room_y1, room_z0),
        (room_x0, room_y0, room_z1),
        (room_x1, room_y0, room_z1),
        (room_x1, room_y1, room_z1),
        (room_x0, room_y1, room_z1),
    ]
    faces = [
        (0, 1, 2, 3),
        (4, 5, 6, 7),
        (0, 1, 5, 4),
        (1, 2, 6, 5),
        (2, 3, 7, 6),
        (3, 0, 4, 7),
    ]
    for face in faces:
        face_pts = np.asarray([vertices[i] for i in face], dtype=float)
        x = face_pts[:, 0]
        y = face_pts[:, 1]
        z = face_pts[:, 2]
        xs, ys = np.meshgrid(x, y)
        zs = np.full_like(xs, z[0], dtype=float)
        ax.plot_surface(xs, ys, zs, alpha=0.12, color='lightgray')

    # ceiling plane and label
    xg, yg = np.meshgrid(np.linspace(room_x0, room_x1, 4), np.linspace(room_y0, room_y1, 4))
    ax.plot_surface(xg, yg, np.full_like(xg, room_z1), alpha=0.25, color='skyblue', linewidth=0)
    ax.text(room_x1 * 0.5, room_y1 * 0.5, room_z1 + 0.1, 'plafond', color='navy', fontsize=10)

    # focal point at the requested position near the room's far side
    focal_point = np.array([12.0, 4.0, 0.0], dtype=float)
    ax.scatter(*focal_point, color='magenta', s=80, label='point focal')

    # mirror, sun and room objects
    mirror = np.array([0.8, 2.5, 0.0], dtype=float)
    plant_center = np.array([3.5, 2.5, 0.5], dtype=float)
    plant_radius = 0.55
    sun = horz2cart(sun_az_deg, sun_el_deg)

    ax.scatter(*mirror, color='orange', s=80, label='miroir')
    ax.scatter(*plant_center, color='green', s=80, label='plante')
    ax.quiver(0, 0, 0, sun[0], sun[1], sun[2], color='red', linewidth=2, label='soleil fixe')

    # all target vectors and normals start from the focal point at the center of the floor
    for row in rows:
        target_vec = np.asarray(horz2cart(row['target_az'], row['target_el']), dtype=float)
        target_vec = 3.0 * target_vec / np.linalg.norm(target_vec)
        norm_vec = np.asarray(normal_from_angles(sun_az_deg, sun_el_deg, row['target_az'], row['target_el']), dtype=float)
        norm_vec = 2.2 * norm_vec / np.linalg.norm(norm_vec)

        ax.quiver(focal_point[0], focal_point[1], focal_point[2], target_vec[0], target_vec[1], target_vec[2], color='black', linewidth=0.8, alpha=0.45)
        ax.quiver(focal_point[0], focal_point[1], focal_point[2], norm_vec[0], norm_vec[1], norm_vec[2], color='blue', linewidth=0.7, alpha=0.5)

        # ceiling intersections for every target direction
        if abs(target_vec[2]) > 1e-12:
            t = (room_z1 - focal_point[2]) / target_vec[2]
            if t > 0:
                ceiling_pt = focal_point + t * target_vec
                ax.scatter(*ceiling_pt, color='orange', s=40, alpha=0.95)
                ax.plot([focal_point[0], ceiling_pt[0]], [focal_point[1], ceiling_pt[1]], [focal_point[2], ceiling_pt[2]], color='orange', linewidth=0.8, alpha=0.8)

    sun_vec = 2.2 * sun / np.linalg.norm(sun)
    ax.quiver(focal_point[0], focal_point[1], focal_point[2], sun_vec[0], sun_vec[1], sun_vec[2], color='red', linewidth=2, label='soleil fixe')

    # start target mark
    start_target = horz2cart(TARGET_AZ, TARGET_EL)
    ax.scatter(start_target[0], start_target[1], start_target[2], color='green', s=40, label='départ cible')

    ax.set_xlim(-3.0, 15.0)
    ax.set_ylim(-2.0, 10.0)
    ax.set_zlim(-0.2, 3.2)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Pièce 3D : miroir, plafond et intersections des vecteurs')
    ax.legend(loc='upper right', fontsize='small')

    plt.tight_layout()
    fig.savefig(ROOT / 'plot_norm_square_fixed_sun.png', dpi=180)
    print(f'saved {ROOT / "plot_norm_square_fixed_sun.png"}')


# Graphique 1 : cible fixe
plot_fixed_target_validation(rows)

# Graphique 2 : soleil fixe à 15° et cible en carré autour du même point de départ que le scénario 1
fixed_sun_rows = scan_target_square_around_fixed_sun(
    sun_az_deg=15.0,
    sun_el_deg=15.0,
    start_target_az=TARGET_AZ,
    start_target_el=TARGET_EL,
    half_span_deg=50.0,
    step_deg=10.0,
)
plot_fixed_sun_square_validation(fixed_sun_rows, sun_az_deg=15.0, sun_el_deg=15.0)

# texte de validation du second scénario
print('\nValidation du scénario 2 : soleil fixe à 15° et cible en carré 50° de côté par pas de 10°')
print('--------------------------------------------------------------------------------------')
print(f"{'target_az':>10} | {'target_el':>10} | {'az_offset':>10} | {'el_offset':>10} | {'dist_sun_norm':>14} | {'dist_target_norm':>16} | {'delta_dist':>11}")
for row in fixed_sun_rows:
    print(
        f"{row['target_az']:>10.2f} | "
        f"{row['target_el']:>10.2f} | "
        f"{row['az_offset']:>10.2f} | "
        f"{row['el_offset']:>10.2f} | "
        f"{row['dist_sun_norm']:>14.6f} | "
        f"{row['dist_target_norm']:>16.6f} | "
        f"{row['delta_dist']:>11.6e}"
    )
print('--------------------------------------------------------------------------------------')
