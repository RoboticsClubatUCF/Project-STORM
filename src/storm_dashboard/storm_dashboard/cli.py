"""Agent-friendly command client for the running dashboard."""

import argparse
import json
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def main():
    parser = argparse.ArgumentParser(description="Send a structured command to the local STORM console")
    parser.add_argument("name", help="Command name, or schema")
    parser.add_argument("args", nargs="?", default="{}", help="JSON object")
    parser.add_argument("--port", type=int, default=8765)
    options = parser.parse_args()
    try:
        if options.name == "schema":
            request = Request(f"http://127.0.0.1:{options.port}/api/schema")
        else:
            values = json.loads(options.args)
            request = Request(f"http://127.0.0.1:{options.port}/api/commands", method="POST",
                              headers={"Content-Type": "application/json"},
                              data=json.dumps({"command": options.name, "args": values}, allow_nan=False).encode())
        with urlopen(request, timeout=45) as response:
            print(json.dumps(json.load(response), indent=2))
    except HTTPError as exc:
        parser.exit(1, exc.read().decode() + "\n")
    except (URLError, ValueError) as exc:
        parser.exit(1, f"Command failed: {exc}\n")
