#!/usr/bin/env python3
"""Fetch public USGS terrain around MDRS and generate a metric Gazebo mesh."""

import argparse
import hashlib
import io
import json
import math
from pathlib import Path
from urllib.parse import urlencode
from urllib.request import urlopen
import numpy as np
from PIL import Image

SERVICE = (
    "https://elevation.nationalmap.gov/arcgis/rest/services/3DEPElevation/ImageServer"
)
LAT = 38 + 24 / 60 + 23.25 / 3600
LON = -(110 + 47 / 60 + 30.85 / 3600)


def utm12(lat, lon):
    """WGS84 transverse Mercator, UTM zone 12 north (EPSG:32612)."""
    a, e2, k = 6378137.0, 0.0066943799901413165, 0.9996
    p, lon_rad, l0 = map(math.radians, (lat, lon, -111.0))
    ep2 = e2 / (1 - e2)
    n = a / math.sqrt(1 - e2 * math.sin(p) ** 2)
    t, c = math.tan(p) ** 2, ep2 * math.cos(p) ** 2
    aa = math.cos(p) * (lon_rad - l0)
    m = a * (
        (1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256) * p
        - (3 * e2 / 8 + 3 * e2**2 / 32 + 45 * e2**3 / 1024) * math.sin(2 * p)
        + (15 * e2**2 / 256 + 45 * e2**3 / 1024) * math.sin(4 * p)
        - (35 * e2**3 / 3072) * math.sin(6 * p)
    )
    east = (
        k
        * n
        * (
            aa
            + (1 - t + c) * aa**3 / 6
            + (5 - 18 * t + t * t + 72 * c - 58 * ep2) * aa**5 / 120
        )
        + 500000
    )
    north = k * (
        m
        + n
        * math.tan(p)
        * (
            aa * aa / 2
            + (5 - t + 9 * c + 4 * c * c) * aa**4 / 24
            + (61 - 58 * t + t * t + 600 * c - 330 * ep2) * aa**6 / 720
        )
    )
    return east, north


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--refresh", action="store_true")
    args = parser.parse_args()
    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    east, north = utm12(LAT, LON)
    span, samples = 1024.0, 257
    bbox = [east - span / 2, north - span / 2, east + span / 2, north + span / 2]
    query = dict(
        bbox=",".join(map(str, bbox)),
        bboxSR=32612,
        imageSR=32612,
        size=f"{samples},{samples}",
        format="tiff",
        pixelType="F32",
        interpolation="RSP_BilinearInterpolation",
        renderingRule=json.dumps({"rasterFunction": "None"}),
        f="json",
    )
    request_url = SERVICE + "/exportImage?" + urlencode(query)
    source = output / "mdrs_dem.tif"
    response_path = output / "usgs_export.json"
    if args.refresh or not source.exists():
        print("Requesting USGS elevation raster...", flush=True)
        with urlopen(request_url, timeout=60) as response:
            export = json.load(response)
        if "error" in export:
            raise RuntimeError(export["error"])
        response_path.write_text(json.dumps(export, indent=2) + "\n")
        # Some service output-directory URLs intermittently return 502.
        # f=image streams the identical export without that extra download.
        with urlopen(
            SERVICE + "/exportImage?" + urlencode(dict(query, f="image")), timeout=60
        ) as response:
            data = response.read()
        # Validate before writing the cache.
        Image.open(io.BytesIO(data)).load()
        source.write_bytes(data)
    export = json.loads(response_path.read_text())
    image = Image.open(source)
    z = np.asarray(image, dtype=np.float64)
    if (
        z.shape != (samples, samples)
        or not np.isfinite(z).all()
        or z.min() < 0
        or z.max() > 5000
    ):
        raise RuntimeError(
            f"Invalid elevation raster: shape={z.shape}, range={z.min(), z.max()}"
        )
    if z.max() - z.min() < 1:
        raise RuntimeError(
            "Elevation range implausibly flat; service may have returned rendered values"
        )
    extent = export["extent"]
    dx = (extent["xmax"] - extent["xmin"]) / samples
    dy = (extent["ymax"] - extent["ymin"]) / samples
    xs = extent["xmin"] + (np.arange(samples) + 0.5) * dx - east
    ys = extent["ymax"] - (np.arange(samples) + 0.5) * dy - north
    reference = float(z[samples // 2, samples // 2])
    # Vertex positions are explicit: image rows north-to-south, x grid east,
    # y grid north. This avoids heightmap orientation and 8-bit quantization.
    x, y = np.meshgrid(xs, ys)
    vertices = np.column_stack((x.ravel(), y.ravel(), (z - reference).ravel()))
    dzdy, dzdx = np.gradient(z, -dy, dx)
    normals = np.column_stack((-dzdx.ravel(), -dzdy.ravel(), np.ones(z.size)))
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    idx = np.arange(samples * samples).reshape(samples, samples)
    a, b, c, d = idx[:-1, :-1], idx[:-1, 1:], idx[1:, :-1], idx[1:, 1:]
    triangles = np.concatenate(
        (
            np.stack((a, c, b), axis=-1).reshape(-1, 3),
            np.stack((b, c, d), axis=-1).reshape(-1, 3),
        )
    )
    values = " ".join(f"{v:.6f}" for v in vertices.ravel())
    normal_values = " ".join(f"{v:.8f}" for v in normals.ravel())
    faces = " ".join(str(v) for v in np.repeat(triangles.ravel(), 2))
    dae = f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><contributor><author>USGS 3DEP; conversion by Project STORM</author></contributor><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
<library_geometries><geometry id="terrain" name="MDRS USGS terrain"><mesh>
<source id="positions"><float_array id="positions-array" count="{vertices.size}">{values}</float_array><technique_common><accessor source="#positions-array" count="{len(vertices)}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<source id="normals"><float_array id="normals-array" count="{normals.size}">{normal_values}</float_array><technique_common><accessor source="#normals-array" count="{len(normals)}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
<triangles count="{len(triangles)}"><input semantic="VERTEX" source="#vertices" offset="0"/><input semantic="NORMAL" source="#normals" offset="1"/><p>{faces}</p></triangles>
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="Scene"><node id="terrain-node"><instance_geometry url="#terrain"/></node></visual_scene></library_visual_scenes><scene><instance_visual_scene url="#Scene"/></scene></COLLADA>'''
    (output / "mdrs_terrain.dae").write_text(dae)
    Image.fromarray(
        np.round((z - z.min()) / (z.max() - z.min()) * 65535).astype(np.uint16)
    ).save(output / "mdrs_height_preview.png")
    metadata = dict(
        site="Mars Desert Research Station, University Rover Challenge venue",
        source=SERVICE,
        request_url=request_url,
        download_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        fetched_utc=__import__("datetime")
        .datetime.now(__import__("datetime").timezone.utc)
        .isoformat(),
        station_latitude=LAT,
        station_longitude=LON,
        projected_crs="EPSG:32612",
        origin_utm_easting=east,
        origin_utm_northing=north,
        origin_source_elevation_m=reference,
        extent=extent,
        requested_span_m=span,
        mesh_span_m=[float(xs[-1] - xs[0]), float(ys[0] - ys[-1])],
        grid_samples=samples,
        resampled_grid_spacing_m=[dx, dy],
        elevation_min_m=float(z.min()),
        elevation_max_m=float(z.max()),
        vertex_count=len(vertices),
        triangle_count=len(triangles),
        axes="x=UTM grid east, y=UTM grid north, z=source elevation minus center elevation",
        limits=[
            "Bare-earth DEM; no buildings, individual rocks, soil traction, or official mission layout.",
            "Grid is resampled; grid spacing is not a claim of native source resolution.",
            "Vertical scale is 1:1; UTM grid north differs slightly from true north.",
            "Source elevation datum is preserved as provided by the service, not ellipsoidal GNSS height.",
        ],
        attribution="USGS National Map 3D Elevation Program (3DEP). Public domain.",
        coordinate_source="https://mdrs2013.marssociety.org/home/crew-162/0126-evareport",
        urc_resources="https://urc.marssociety.org/home/resources",
    )
    (output / "provenance.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(
        json.dumps(
            {
                k: metadata[k]
                for k in [
                    "origin_utm_easting",
                    "origin_utm_northing",
                    "origin_source_elevation_m",
                    "elevation_min_m",
                    "elevation_max_m",
                    "resampled_grid_spacing_m",
                    "triangle_count",
                ]
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
