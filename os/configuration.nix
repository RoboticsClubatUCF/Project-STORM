{ pkgs, ... } : {
  imports = [
    ./disk-layout.nix
  ];

  nix.nixPath = [
    "nixos-config=/etc/nixos/configuration.nix"
    "nixpkgs=/nix/var/nix/profiles/per-user/root/channels/nixpkgs"
  ];

  boot.loader.grub.device = "nodev";
  boot.loader.grub.efiSupport = true;
  boot.loader.grub.efiInstallAsRemovable = true;
  boot.loader.efi.efiSysMountPoint = "/boot";
  boot.loader.efi.canTouchEfiVariables = false;

  hardware.nvidia-jetpack.enable = true;
  hardware.nvidia-jetpack.som = "orin-nano"; # Other options include orin-agx, xavier-nx, and xavier-nx-emmc
  hardware.nvidia-jetpack.super = true;
  hardware.nvidia-jetpack.carrierBoard = "devkit";

  # Fixes a known issue with the image builder falling back on the AGX based power config for orin nano super instead of the proper config for the orin nano super
  services.nvpmodel.configFile = "${pkgs.nvidia-jetpack.l4t-nvpmodel}/etc/nvpmodel/nvpmodel_p3767_0003_super.conf";

  users.users.nixos = {
    isNormalUser = true;
    extraGroups = [
      "wheel"
      "networkmanager"
      "video"
      "docker"
      "podman"
    ];
    # Allow the graphical user to login without password
    initialPassword = "nixos123";

 };

  services.openssh.enable = true;

  users.users.nixos.openssh.authorizedKeys.keys = [
    # YOUR SSH PUB KEY HERE #
    "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABgQCpv4hyo6JLrJQ80rAIb0tQfNro323iyY4YwV/Dpvt88JlnzUGvX160NqTbKZnGkdSrJqtOvGya2aGhdnj/5Wi8PnFoV+E1kWSoyW0dVMEukWrR6pIEzoxEApQ12ROFqsCK9N6IWVB6JHlNH0sqglyQL025WE89OQZEJy2E4w2kAmuRZfYxhMWJkp7cFn7NefMh6YG1J6ZZ75lKcQN65wiJ2sH/d6ujFVS6cX98kU2qkibJ8BuyD83RYBl8U0SoiD1MLd7LqDYHkPEm68mJT/ewP7WgdPPoKuj2+YAKr8Z6DTtHzDW9oehIwxfNssum/ZuBqBKHcfCUPfz1TC8habuxIo5vhgcB/9o/424DPQDzWcdURbBrEw4Tx7Y3ykiyMJ4Z5QAL9RDac+mju7RvIyXt51kXChBSenEMhu7hzFcUIGaXEBnRlu8eaH2da6eebDWc2nEDhG4TU3sb4YA/P9QFdW+JCyS9t7D90UgjhIan3u0kNzzbsieiMc9DETy8sak="
    "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAACAQCkeQ0MSty9182D2XX5mqaN/NF9ijHh8G04BxOkEKM9ABLf0ytQBFDSjHONoYemQgaLRw+dMWW9tNuHC7/6I4PlEOSzJHBlWCjVDwC/D7koT4MhSJAKCFYVCOK4hvf6gf+MkZKaJcbTlNpDHje3WB//emoMvMfht1Eazl6nIUQSeI8GVSOQb8eWVDFHcQSzDAPZejce3McWs9Dl8ILFN123TDXq5n0qzukq0yi5O3U+DzUIXMG7A6V0vuZ1Juks4Jg7J+LBx2X3cLpJ5/s4Gm66DDPGoDmnQHXUFfVqwDq11LeNjl4lNxQxvnq4aamVItWy+pd52FsC/pWswx5M2PO/5hADUuO0RUpQzkvCg3kXGklZ8BdC5vP5j4y5TvT8LrIhEl8O+kqBcWTFonvyoSK8s7xyOZLOk5nzVep/qdnhOsyOMb8hM/xUhyPER4zxHZF9yAjBdyd2OYxqUh09x2zE/jhqYdplEv4Zw2Kt9bSzI1+2VR0lKzBRVW1D4yV96UW7xFKpFi5/1krZm23xG4lKZ7o6Tbv/4+3c6njljv3WiQuNBxligbU6qBXxgIqomDw0ZrZ18DXcXQ8oIuf0FrLerBQdk0H7BcQWcL+6DJOBml/yGi9U4jmw91fdSf1ZVIWx/f3H4yuXGD6EACJBMw5Sdqlzq1q6pMzQ0E0f/6r3rQ=="
  ];
  users.users.root.openssh.authorizedKeys.keys = [
    # YOUR SSH PUB KEY HERE #
    "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABgQCpv4hyo6JLrJQ80rAIb0tQfNro323iyY4YwV/Dpvt88JlnzUGvX160NqTbKZnGkdSrJqtOvGya2aGhdnj/5Wi8PnFoV+E1kWSoyW0dVMEukWrR6pIEzoxEApQ12ROFqsCK9N6IWVB6JHlNH0sqglyQL025WE89OQZEJy2E4w2kAmuRZfYxhMWJkp7cFn7NefMh6YG1J6ZZ75lKcQN65wiJ2sH/d6ujFVS6cX98kU2qkibJ8BuyD83RYBl8U0SoiD1MLd7LqDYHkPEm68mJT/ewP7WgdPPoKuj2+YAKr8Z6DTtHzDW9oehIwxfNssum/ZuBqBKHcfCUPfz1TC8habuxIo5vhgcB/9o/424DPQDzWcdURbBrEw4Tx7Y3ykiyMJ4Z5QAL9RDac+mju7RvIyXt51kXChBSenEMhu7hzFcUIGaXEBnRlu8eaH2da6eebDWc2nEDhG4TU3sb4YA/P9QFdW+JCyS9t7D90UgjhIan3u0kNzzbsieiMc9DETy8sak="
    "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAACAQCkeQ0MSty9182D2XX5mqaN/NF9ijHh8G04BxOkEKM9ABLf0ytQBFDSjHONoYemQgaLRw+dMWW9tNuHC7/6I4PlEOSzJHBlWCjVDwC/D7koT4MhSJAKCFYVCOK4hvf6gf+MkZKaJcbTlNpDHje3WB//emoMvMfht1Eazl6nIUQSeI8GVSOQb8eWVDFHcQSzDAPZejce3McWs9Dl8ILFN123TDXq5n0qzukq0yi5O3U+DzUIXMG7A6V0vuZ1Juks4Jg7J+LBx2X3cLpJ5/s4Gm66DDPGoDmnQHXUFfVqwDq11LeNjl4lNxQxvnq4aamVItWy+pd52FsC/pWswx5M2PO/5hADUuO0RUpQzkvCg3kXGklZ8BdC5vP5j4y5TvT8LrIhEl8O+kqBcWTFonvyoSK8s7xyOZLOk5nzVep/qdnhOsyOMb8hM/xUhyPER4zxHZF9yAjBdyd2OYxqUh09x2zE/jhqYdplEv4Zw2Kt9bSzI1+2VR0lKzBRVW1D4yV96UW7xFKpFi5/1krZm23xG4lKZ7o6Tbv/4+3c6njljv3WiQuNBxligbU6qBXxgIqomDw0ZrZ18DXcXQ8oIuf0FrLerBQdk0H7BcQWcL+6DJOBml/yGi9U4jmw91fdSf1ZVIWx/f3H4yuXGD6EACJBMw5Sdqlzq1q6pMzQ0E0f/6r3rQ=="
  ];

  services.getty.autologinUser = "nixos";

  networking = {
    firewall.enable = true;
    hostName = "curiosity";
    useDHCP = false;
    interfaces.eth0 = {
      ipv4.addresses = [ {
        address = "192.168.1.64";
        prefixLength = 24;
      } ];
    };
    firewall.allowedTCPPorts = [
      22
      25565
      443
      80
      40111
      8080
      9120
    ];
    defaultGateway = "192.168.1.1";
    nameservers = [ "8.8.8.8" "1.1.1.1" ];
  };

  nixpkgs.config.allowUnfree = true;

  # hardware.nvidia-container-toolkit.enable = true;
  virtualisation.docker = {
    enable = true;
    enableOnBoot = true;
    enableNvidia = true;
  };

  virtualisation.podman = {
    enable = true;
    # dockerCompat = true; # Creates a symlink from docker to podman
    defaultNetwork.settings.dns_enabled = true; # Required for containers under podman-compose to be able to talk to each other.
  };

  services.tailscale = {
    # Enable tailscale at startup
    enable = true;

    # If you would like to use a preauthorized key, set
    # authKeyFile = "/run/secrets/tailscale_key";
    # Note: maximum expire time is 90 days
  };

  nix.settings.experimental-features = [ "nix-command" "flakes" ];
  environment.systemPackages = with pkgs; [
    podman
    podman-tui
    docker
    docker-compose
    git
    neovim
    wget
  ];

  # Enable GPU support - needed even for CUDA and containers
  hardware.graphics.enable = true;

  services.xserver.videoDrivers = [ "nvidia" ];
  hardware.nvidia = {
    modesetting.enable = true;
    open = false;
    nvidiaSettings = true;
    # package = config.boot.kernelPackages.nvidiaPackages.production;
  };
}