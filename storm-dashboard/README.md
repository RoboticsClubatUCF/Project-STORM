# storm-dashboard

Web dashboard for the rover. Connects to the rover's foxglove_bridge
(`storm_basestation/bridge.launch.py`) and shows battery, computer stats and
diagnostics. Lichtblick is still the main tool; this is a lighter view.

`COLCON_IGNORE` keeps colcon out of this folder.

## Local dev

```bash
cp .env.example .env.local     # set NEXT_PUBLIC_ROVER_WS=ws://<rover-ip>:8765
npm install
npm run dev                    # http://localhost:3000
```

## Hosting on the base station Jetson

Laptops on the base station hotspot open `http://10.42.0.1:3000`. Their
browser connects straight to the rover's bridge through the hotspot's NAT, so
the URL in `.env.local` must be the rover's link address, not localhost.

Needs Node 18.17+ (Ubuntu's apt package is too old; use NodeSource or nvm).

```bash
echo "NEXT_PUBLIC_ROVER_WS=ws://192.168.1.60:8765" > .env.local
npm install
npm run build                  # rebuild after changing .env.local
```

Start at boot with the user service in `systemd/storm-dashboard.service`
(install steps are at the top of that file).

## Notes

Messages are decoded in the browser from the schemas the bridge advertises, so
field changes in `storm_interfaces` show up without regenerating anything here.
Only the TypeScript types in `lib/useRover.ts` need to stay in sync.
