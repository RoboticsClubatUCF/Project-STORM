# storm_basestation

Base station side of the rover: joystick for driving, Lichtblick and
storm-dashboard for monitoring.

```
Rover Jetson                 bridge radios           Base Jetson                    Viewing laptop
foxglove_bridge :8765  <──eth── ~~~~~~~~ ──eth──>  joy_node (DDS /joy)
teleop, telemetry                                  hotspot STORM-BASE  <──WiFi──>  Lichtblick / browser
                                                   storm-dashboard :3000
```

The bridge runs on the rover so only one WebSocket crosses the radio link,
carrying just the whitelisted topics. It's read-only: viewers can't publish,
call services or change params. The laptop doesn't need ROS.

## Rover

`ros2 launch storm_teleop teleop.launch.py use_joy:=false` starts the bridge
(`enable_bridge:=false` to skip). Standalone:

```bash
ros2 launch storm_basestation bridge.launch.py
```

Needs `ros-humble-foxglove-bridge` (`rosdep install` picks it up).

## Network

Everything on the radio link has a static address on 192.168.1.0/24, listed
in `config/network.env`:

| Device | IP |
|--------|----|
| Rover Jetson | 192.168.1.50 |
| Base Jetson | 192.168.1.60 |
| Rover radio | 192.168.1.21 |
| Base radio | 192.168.1.20 |

The radios are a transparent link, so set the Jetsons' Ethernet ports to
their addresses. On the rover:

```bash
sudo nmcli con add type ethernet ifname eth0 con-name storm-link \
  ipv4.method manual ipv4.addresses 192.168.1.50/24
```

On the base Jetson, `scripts/link_up.sh` creates the same `storm-link`
profile from `network.env`, routes multicast out the radio port and keeps the
default route on WiFi. The Docker setup runs it on every `up`; natively:

```bash
sudo scripts/link_up.sh
```

It's safe to re-run and only bounces the link when something changed. If
the Jetson has more than one Ethernet port, set `LINK_IFACE` in
`network.env` (`nmcli dev` lists them). If an IP changes, update
`config/network.env` and `config/fastdds_peers.xml`.

Then start the hotspot on the base Jetson's WiFi (once; it comes back at boot):

```bash
sudo PASSWORD=<8+ chars> scripts/hotspot_up.sh            # 2.4 GHz on wlan0
sudo PASSWORD=<8+ chars> BAND=a scripts/hotspot_up.sh     # 5 GHz
```

Laptops on `STORM-BASE` get a 10.42.0.x address and are NATed to the rover, so
`ws://192.168.1.50:8765` works from the laptop as-is.

### Checking the link

With the rover running, check every layer from the base Jetson (interface,
multicast, ping to each radio and the rover, bridge port, DDS discovery of
the rover's teleop node):

```bash
scripts/check_link.sh
docker compose run --rm basestation /ws/src/storm_basestation/scripts/check_link.sh   # in Docker
```

The basestation container also runs it at startup; see
`docker compose logs basestation`.

### DDS discovery without multicast

If `check_link.sh` reports that the radios drop multicast, either enable multicast
pass-through in the radio settings, or use unicast peers on **both** machines:

```bash
echo "export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/Project-STORM/src/storm_basestation/config/fastdds_peers.xml" >> ~/.bashrc
source ~/.bashrc
ros2 daemon stop
```

Then restart any running launch. `config/fastdds_peers.xml` lists the two
Jetsons from `config/network.env`.

## Base Jetson

Drive with the joystick plugged into the base Jetson (no build needed):

```bash
export ROS_DOMAIN_ID=42   # same as the rover
ros2 launch /path/to/src/storm_basestation/launch/base_station.launch.py
```

Needs ROS 2 Humble (JetPack 6 / Ubuntu 22.04). If the link drops,
`cmd_vel_timeout` stops the rover after 0.5 s.

To serve storm-dashboard to laptops, see `storm-dashboard/README.md`.

### Docker

Instead of the launch above and the dashboard's systemd unit, the base Jetson
can run both in containers that restart at boot. A one-shot `netsetup`
container runs `link_up.sh` first through the host's NetworkManager:

```bash
cd .devcontainer/basestation
printf 'ROS_DOMAIN_ID=42\nNEXT_PUBLIC_ROVER_WS=ws://192.168.1.50:8765\n' > .env
docker compose up -d --build     # rebuild after changing .env or the code
```

Both values above are the defaults, so `.env` is only needed to change them.
Uncomment `FASTRTPS_DEFAULT_PROFILES_FILE` in `docker-compose.yml` if DDS
needs unicast peers. Disable the systemd unit first so port 3000 is free.

The same folder is a VS Code dev container (Reopen in Container >
STORM Base Station) with ROS and Node for working on this package and the
dashboard.

## Viewing laptop

Join `STORM-BASE`, then either:

- **Dashboard:** open `http://10.42.0.1:3000`.
- **Lichtblick:** install the
  [desktop app](https://github.com/lichtblick-suite/lichtblick/releases), Open
  connection > **Foxglove WebSocket** > `ws://192.168.1.50:8765`, then Layouts >
  Import from file > `lichtblick/storm_layout.json`.

## Files

| File | What it does |
|------|--------------|
| `config/foxglove_bridge.yaml` | Port, topic whitelist, read-only capabilities |
| `config/network.env` | Static IPs of the Jetsons and radios |
| `config/fastdds_peers.xml` | Unicast DDS discovery between rover and base station |
| `launch/bridge.launch.py` | foxglove_bridge (rover) |
| `launch/base_station.launch.py` | joy_node with `storm_teleop/config/joy_teleop.yaml` (base Jetson) |
| `lichtblick/storm_layout.json` | 3D + URDF, diagnostics, battery and computer plots |
| `scripts/hotspot_up.sh` | WiFi hotspot on the base Jetson, NAT to the rover |
| `scripts/link_up.sh` | Static IP on the base Jetson's radio port (NetworkManager) |
| `scripts/check_link.sh` | Checks the link, multicast and DDS discovery of the rover |

To show another topic in Lichtblick, add it to `topic_whitelist`.
