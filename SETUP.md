# Rover and base station setup

One-time setup for a new rover or base station Jetson (JetPack 6, Ubuntu 22.04).
Commands assume the repo is cloned to `~/Project-STORM`.

| Device | IP |
|--------|----|
| Base radio | 192.168.1.20 |
| Rover radio | 192.168.1.21 |
| Base station Jetson | 192.168.1.50 |
| Rover Jetson | 192.168.1.60 |

Addresses live in `src/storm_basestation/config/network.env`. If one changes,
update that file and `src/storm_basestation/config/fastdds_peers.xml`.

## Rover Jetson

**1. Static IP** on the Ethernet port to the radio (`nmcli dev` shows the port
name; it isn't always `eth0`):

```bash
sudo nmcli con add type ethernet ifname eth0 con-name storm-link \
  ipv4.method manual ipv4.addresses 192.168.1.60/24
sudo nmcli con up storm-link
```

**2. Environment, groups and SSH** (log out and back in afterwards):

```bash
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
sudo usermod -aG input $USER          # joy_watchdog reads the controller's motion sensor
sudo systemctl enable --now ssh
```

**3. sparkcan** (skip if already installed):

```bash
git clone https://github.com/grayson-arendt/sparkcan.git ~/sparkcan
cd ~/sparkcan && cmake -S . -B build && cmake --build build && sudo cmake --install build
```

**4. Build:**

```bash
git clone https://github.com/RoboticsClubatUCF/Project-STORM.git ~/Project-STORM
cd ~/Project-STORM
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
echo 'source ~/Project-STORM/install/setup.bash' >> ~/.bashrc && source ~/.bashrc
```

**5. CAN at boot:**

```bash
cd ~/Project-STORM/src/storm_teleop
sudo cp systemd/storm-can0.service /etc/systemd/system/
sudo cp scripts/can0_up.sh /usr/local/bin/ && sudo chmod +x /usr/local/bin/can0_up.sh
sudo systemctl daemon-reload && sudo systemctl enable --now storm-can0.service
```

**6. Bluetooth controller failsafe** (only if a controller pairs directly to the rover):

```bash
sudo apt install bluez
sudo cp scripts/bt_joy_failsafe.sh /usr/local/bin/ && sudo chmod +x /usr/local/bin/bt_joy_failsafe.sh
sudo cp udev/99-storm-bt-joy.rules /etc/udev/rules.d/ && sudo udevadm control --reload
```

**7. Base layer at boot** (bridge, telemetry and the dashboard's teleop
launcher; teleop itself never starts on its own):

```bash
mkdir -p ~/.config/systemd/user
cp ~/Project-STORM/src/storm_teleop/systemd/storm-rover-base.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now storm-rover-base.service
sudo loginctl enable-linger $USER
```

**Each session:** open the dashboard, pick Teleop or SpaceU, tick
"Controller on rover" if the controller is paired to the rover, and press
Start. Logs: `journalctl --user -u storm-rover-base -f`.

To launch by hand instead, stop the base layer first (otherwise two bridges
fight over port 8765):

```bash
systemctl --user stop storm-rover-base.service
ros2 launch storm_teleop teleop.launch.py use_joy:=false   # controller on the base station
ros2 launch storm_teleop teleop.launch.py                  # controller paired to the rover
```

## Base station Jetson

**1. Docker** (log out and back in after `usermod`):

```bash
docker compose version                  # if missing, install Docker Engine + compose plugin
sudo usermod -aG docker $USER
sudo systemctl enable --now docker
```

**2. Clone and build** while the Jetson still has internet over WiFi:

```bash
git clone https://github.com/RoboticsClubatUCF/Project-STORM.git ~/Project-STORM
nmcli dev     # more than one ethernet device? set LINK_IFACE in src/storm_basestation/config/network.env
cd ~/Project-STORM/.devcontainer/basestation && docker compose build
```

**3. Hotspot** (once; it comes back at boot). The WiFi becomes the hotspot, so
the Jetson loses internet after this:

```bash
sudo PASSWORD=<8+ chars> ~/Project-STORM/src/storm_basestation/scripts/hotspot_up.sh
```

**4. Start.** Cable the Ethernet port to the base radio, then:

```bash
cd ~/Project-STORM/.devcontainer/basestation
docker compose up -d
```

This sets the static IP (192.168.1.50), runs the link check, then starts
joy_node with the watchdog and the dashboard. All of it restarts at boot.

## Check

With the rover running `teleop.launch.py use_joy:=false`, on the base station:

```bash
cd ~/Project-STORM/.devcontainer/basestation
docker compose run --rm basestation /ws/src/storm_basestation/scripts/check_link.sh
docker compose logs basestation | grep -i watching    # watchdog found the controller
```

Plug the controller into the base station and drive. Laptops join `STORM-BASE`
and open `http://10.42.0.1:3000`.

If `check_link.sh` says the radios drop multicast, enable multicast
pass-through in the radio settings. If that isn't possible, use unicast peers:

- Rover: `echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=~/Project-STORM/src/storm_basestation/config/fastdds_peers.xml' >> ~/.bashrc`
- Base station: uncomment `FASTRTPS_DEFAULT_PROFILES_FILE` in
  `.devcontainer/basestation/docker-compose.yml`, then `docker compose up -d`

## Troubleshooting

| Symptom | Check |
|---------|-------|
| Base station containers won't start | `docker compose ps -a`, `docker compose logs netsetup basestation` |
| No link to the rover | `check_link.sh` shows the first broken hop |
| Can't SSH to the rover | `ip -4 addr` on the base station should show 192.168.1.50; then `ssh <user>@192.168.1.60` |
| Rover keeps driving after the controller dies | `Watching /dev/input/eventN` missing from the launch log (user not in `input` group?) |

Package details: `src/storm_teleop/README.md`, `src/storm_basestation/README.md`,
`storm-dashboard/README.md`.
