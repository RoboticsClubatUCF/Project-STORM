final: prev:
{
  storm-arm = final.callPackage ././src/storm_arm/package.nix {};
  storm-bringup = final.callPackage ././src/storm_bringup/package.nix {};
  storm-control = final.callPackage ././src/storm_control/package.nix {};
  storm-dashboard = final.callPackage ././src/storm_dashboard/package.nix {};
  storm-description = final.callPackage ././src/storm_description/package.nix {};
  storm-gazebo = final.callPackage ././src/storm_gazebo/package.nix {};
  storm-hardware = final.callPackage ././src/storm_hardware/package.nix {};
  storm-mission = final.callPackage ././src/storm_mission/package.nix {};
  storm-navigation = final.callPackage ././src/storm_navigation/package.nix {};
  storm-perception = final.callPackage ././src/storm_perception/package.nix {};
  storm-sensors = final.callPackage ././src/storm_sensors/package.nix {};
  storm-sparkcan-vendor = final.callPackage ././src/storm_sparkcan_vendor/package.nix {};
  storm-teleop = final.callPackage ././src/storm_teleop/package.nix {};
}
