#!/usr/bin/env python3
"""WGS84 (lat, lon, alt) to ``map`` ENU ``PoseStamped`` list; origin defines local tangent plane."""

import csv
import math
from pathlib import Path
from typing import List

from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header

# sources: https://gssc.esa.int/navipedia/index.php?title=Ellipsoidal_and_Cartesian_Coordinates_Conversion&action=edit
# sources: https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates
# sources: https://en.wikipedia.org/wiki/World_Geodetic_System


def degrees_to_radians(lat: float, lon: float) -> tuple:
    return math.radians(lat), math.radians(lon)


def wgs84_to_ecef(lat_rad: float, lon_rad: float, alt_m: float) -> tuple:
    """Geodetic (radians, meters) to ECEF (m)."""
    a = 6378137.0
    f = 1.0 / 298.257223563
    e_sq = 2.0 * f - f**2
    n = a / math.sqrt(1.0 - e_sq * math.sin(lat_rad) ** 2)
    x = (n + alt_m) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (n + alt_m) * math.cos(lat_rad) * math.sin(lon_rad)
    z = ((1.0 - e_sq) * n + alt_m) * math.sin(lat_rad)
    return x, y, z


def diff_ecef_to_enu(lat0_rad: float, lon0_rad: float, dx: float, dy: float, dz: float) -> tuple:
    """ECEF delta (dx,dy,dz) to ENU at origin (lat0, lon0 in radians)."""
    sin_lat = math.sin(lat0_rad)
    cos_lat = math.cos(lat0_rad)
    sin_lon = math.sin(lon0_rad)
    cos_lon = math.cos(lon0_rad)
    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return east, north, up


def make_pose(x: float, y: float, z: float) -> PoseStamped:
    return PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=x, y=y, z=z)))


def process_coordinates(csv_filepath: str | Path, lat0: float, lon0: float, alt0: float) -> List[PoseStamped]:
    """
    CSV with header row: lat,lon,alt (degrees, degrees, meters).
    Same vertical datum for alt as for alt0.
    """
    path = Path(csv_filepath)
    waypoints_enu: List[PoseStamped] = []

    lat0_rad, lon0_rad = degrees_to_radians(lat0, lon0)
    x0, y0, z0 = wgs84_to_ecef(lat0_rad, lon0_rad, alt0)

    with path.open(newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f'Missing CSV header in {path}')
        fields = {h.strip().lower() for h in reader.fieldnames if h}
        if not {'lat', 'lon', 'alt'}.issubset(fields):
            raise ValueError(
                f'CSV must have columns lat,lon,alt (header row). Got: {reader.fieldnames!r}'
            )

        for row in reader:
            if not row or not any((v or '').strip() for v in row.values()):
                continue
            lat_raw = (row.get('lat') or '').strip()
            if lat_raw.startswith('#'):
                continue
            lat = float(lat_raw)
            lon = float(row['lon'].strip())
            alt = float(row['alt'].strip())
            lat_rad, lon_rad = degrees_to_radians(lat, lon)
            x, y, z = wgs84_to_ecef(lat_rad, lon_rad, alt)
            dx, dy, dz = x - x0, y - y0, z - z0
            e, n, u = diff_ecef_to_enu(lat0_rad, lon0_rad, dx, dy, dz)
            waypoints_enu.append(make_pose(e, n, u))

    if not waypoints_enu:
        raise ValueError(f'No waypoints read from {path}')
    return waypoints_enu
