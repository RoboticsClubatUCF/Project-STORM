# University Rover Challenge site terrain

This world uses **real USGS 3DEP bare-earth elevation data around the Mars Desert
Research Station (MDRS), Utah**, the venue linked by the official URC resources
page. It is a geospatial reconstruction of the venue, **not an official mission
course layout or a survey of task targets**. No official Gazebo world was found.
URC links to lidar through Equator; the import uses USGS's public service directly.

## Run

In `wsl -d Ubuntu-24.04`, stop the previous STORM launch before starting:

```bash
cd ~/Project-STORM-jazzy
bash devscripts/build.sh
source install/setup.bash
export ROS_DOMAIN_ID=42 GZ_PARTITION=storm_sim
ros2 launch storm_bringup simulation.launch.py environment:=urc
# Headless variant:
# ros2 launch storm_bringup simulation.launch.py environment:=urc gui:=false rviz:=false
```

STORM's original six-wheel URDF is used. The example uses ROS domain 42 and the storm_sim Gazebo partition. Additional terminals need:

```bash
source ~/Project-STORM-jazzy/install/setup.bash
export ROS_DOMAIN_ID=42 GZ_PARTITION=storm_sim
```

The URC launch starts a **new SLAM map**, not the small indoor arena map.
Gazebo terrain and a Nav2 occupancy map are different: an elevation raster
defines the physical world; lidar/SLAM build the navigation map while driving.
The URC configuration uses a 40 m rolling global costmap instead of the arena's
static-map layer. It retains unknown space until lidar observes it, treats valid
infinite-range laser readings as clear rays, and uses DWB at up to 0.3 m/s.
This allows local exploration without requiring a complete SLAM map first.
Sparse landmarks can limit 2D SLAM outdoors. Nav2's current 2D costmaps do not
evaluate terrain slope or tip-over risk; terrain testing does not validate
autonomous traversability. The rover has no articulated suspension model.

## Data and coordinates

- Origin at published MDRS habitat coordinates: 38.406458333 N,
  -110.791902778 E (WGS84).
- Horizontal CRS: UTM zone 12N, EPSG:32612. World x points grid east and y grid
  north; grid north differs slightly from true north. Simulated GNSS uses
  the MDRS origin with an approximate altitude reference.
- Origin UTM: E 518169.088, N 4250934.479 m.
- World z=0 corresponds to center raster elevation 1372.995 m
  in the service's source elevation datum. Elevation is relative in simulation;
  do not interpret it as GNSS ellipsoidal altitude.
- Requested area: 1024 x 1024 m. Mesh covers pixel centers, approximately
  1020.016 x 1020.016 m.
- Grid: 257 x 257 points, 3.984436 m spacing.
  This is a resampled export, not a claim of native source resolution.
- Source elevation range: 1363.451 to 1417.621 m.
  Horizontal and vertical scale are both 1:1. Float elevations are preserved in
  the mesh; the PNG is a preview only.
- The same mesh is used for rendering and collision, with rows north to south
  explicitly converted to Cartesian coordinates. Terrain shading is synthetic.
- Buildings, individual rocks, soil deformation, and competition props are absent.
  Friction is a generic parameter, not a measured MDRS soil model.

## Reproduce

Requires Python 3, NumPy, and Pillow (`python3-numpy python3-pil` on Ubuntu).
The original GeoTIFF, generated COLLADA mesh, preview, USGS export response,
source URL, and SHA-256 hash are stored in `src/storm_gazebo/terrain/mdrs/`.
To regenerate from the cached GeoTIFF without network access:

```bash
python3 devscripts/fetch_urc_terrain.py --output src/storm_gazebo/terrain/mdrs
# Add --refresh to request current elevation data from USGS.
```

## Sources and attribution

- URC official resources: https://urc.marssociety.org/home/resources
- Official habitat coordinates: https://mdrs2013.marssociety.org/home/crew-162/0126-evareport
- USGS 3DEP service: https://elevation.nationalmap.gov/arcgis/rest/services/3DEPElevation/ImageServer
- USGS terms: https://www.usgs.gov/3d-elevation-program (free, without use restrictions).

Credit: USGS National Map 3D Elevation Program (3DEP), public domain.
Retrieval details and datum/coordinate limitations: `terrain/mdrs/provenance.json`.

## Runtime validation (2026-10-29)

- Both ROS packages build successfully.
- Source SHA-256, all 66,049 mesh elevations, axis orientation, unit scale,
  per-vertex normal vectors, and launch/config syntax validated.
- Gazebo loaded the 131,072-triangle mesh for both rendering and collision.
- STORM settled at approximately z=0.0015 m at the origin without falling through
  the terrain. Both wheel controllers activated; lidar and odometry published.
- SLAM and all five Nav2 lifecycle nodes activated.
- With the URC rolling costmap and DWB controller, NavigateToPose to (-1, 1)
  returned SUCCEEDED, error_code 0.
- Independent Gazebo ground truth after that action was approximately
  (-1.51, 0.52) m: about 0.70 m from the requested coordinate. This illustrates
  SLAM/odometry drift on sparse terrain; action success is not a claim of
  geographic accuracy. Precise outdoor navigation requires further localization
  work (for example GNSS/IMU fusion) and testing.
- The inherited RViz indexed-map shader warning and occasional sensor-filter
  drops remain. The URC launch runs in Gazebo and RViz on WSLg.

The arena configuration remains available separately. URC uses its own DWB
configuration because the installed RPP controller crashed during a sparse-map
terrain test. The final URC DWB navigation test completed without that crash.
