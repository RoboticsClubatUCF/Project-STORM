{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!

    jetpack.url = "github:anduril/jetpack-nixos/master";
    jetpack.inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs";

    disko.url = "github:nix-community/disko"; # was missing
    disko.inputs.nixpkgs.follows = "nixpkgs";
    nixos-facter-modules.url = "github:numtide/nixos-facter-modules";
  };

  outputs = { self, nix-ros-overlay, nixpkgs, jetpack, disko, nixos-facter-modules, ... }@inputs:
    let
      rosDistro = "jazzy";

      # Hoisted out of eachDefaultSystem so the NixOS system can use them too
      applyDistroOverlay = rosOverlay: rosPackages:
        rosPackages
        // builtins.mapAttrs (
          _: rosPkgs: if rosPkgs ? overrideScope then rosPkgs.overrideScope rosOverlay else rosPkgs
        ) rosPackages;

      rosDistroOverlays = final: prev: {
        rosPackages = applyDistroOverlay (import ./overlay.nix) prev.rosPackages;
      };

      # Order matters: the base overlay must come first
      overlays = [
        nix-ros-overlay.overlays.default
        rosDistroOverlays
      ];
    in
    {
      nixosConfigurations.nixos = nixpkgs.lib.nixosSystem {
        system = "aarch64-linux";
        modules = [
          disko.nixosModules.disko
          ./configuration.nix
          ./hardware-configuration.nix
          jetpack.nixosModules.default

          ({ pkgs, ... }: {
            nixpkgs.overlays = overlays;

            environment.systemPackages = [
              (pkgs.rosPackages.${rosDistro}.buildEnv {
                paths = with pkgs.rosPackages.${rosDistro}; [
                  ros-core
                  # add your own packages from ./overlay.nix here
                ];
              })
            ];
          })
        ];
      };
    }
    // nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system overlays; };
      in {
        legacyPackages = pkgs.rosPackages;
        packages = builtins.intersectAttrs (import ./overlay.nix null null) pkgs.rosPackages.${rosDistro};
        checks = builtins.intersectAttrs (import ./overlay.nix null null) pkgs.rosPackages.${rosDistro};
        devShells.default = import ./shell.nix {
          inherit pkgs rosDistro;
          extraPkgs = { };
          extraPaths = [ ];
        };
      });

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}