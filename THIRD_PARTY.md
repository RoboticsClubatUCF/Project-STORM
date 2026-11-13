# Essential third-party dependencies and data

ROS 2 Jazzy, ros2_control, Nav2, Gazebo Harmonic and SLAM Toolbox are installed
ROS/system dependencies governed by their respective package licenses.

The optional CAN build uses grayson-arendt/sparkcan at commit
11d2d13787fc32e508690926efc3c9749e58770e. The library is MIT licensed. Its license is installed under the vendor
package's share directory. It is not vendored as rover
geometry or a demonstration object.

MDRS elevation data: USGS 3DEP, public domain. Its original raster, mesh,
metadata and attribution are kept only for the explicitly requested URC world.
