#!/usr/bin/env python3
"""Run the dashboard's visible simulator tests through its public command API."""

import argparse
import json
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--test", choices=["sensors", "waypoints", "deadman", "delivery", "all"], default="sensors")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    base = f"http://127.0.0.1:{args.port}"

    def state():
        with urlopen(base + "/api/state", timeout=10) as response:
            return json.load(response)

    def command(name, values=None):
        request = Request(base + "/api/commands", data=json.dumps({
            "command": name, "args": values or {}}).encode(), headers={"Content-Type": "application/json"})
        with urlopen(request, timeout=45) as response:
            return json.load(response)["result"]

    try:
        for test_id in (["sensors", "deadman", "waypoints"] if args.test == "all" else [args.test]):
            current = state()
            if current["busy"] or current["processes"].get("mission", {}).get("running"):
                raise RuntimeError("Cancel the existing test or mission before running this suite")
            if current["processes"].get("simulation", {}).get("running"):
                print("Closing this console's simulator for a fresh test", flush=True)
                command("simulation.stop")
            command("test.run", {"id": test_id})
            print(f"Started visible test: {test_id}", flush=True)
            deadline = time.monotonic() + (750 if test_id == "delivery" else 280)
            previous = None
            while time.monotonic() < deadline:
                result = state()["test"]
                stage = result["status"], result["stage"]
                if stage != previous:
                    print(" / ".join(stage), flush=True)
                    previous = stage
                if result["status"] in ("passed", "failed", "cancelled"):
                    if result["status"] != "passed":
                        raise RuntimeError(json.dumps(result))
                    print(json.dumps(result.get("result", {}), indent=2), flush=True)
                    # Allow the worker's report checkpoint to finish before another run.
                    while state()["busy"]:
                        time.sleep(.1)
                    break
                time.sleep(1)
            else:
                command("test.cancel")
                raise RuntimeError("Test exceeded suite deadline; cancellation requested")
    except (HTTPError, URLError, RuntimeError) as exc:
        parser.exit(1, f"Dashboard test failed: {exc}\n")


if __name__ == "__main__":
    main()
