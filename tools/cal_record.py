#!/usr/bin/env python3
"""Records a compass calibration streamed over WebSocket into a JSON Lines file.

Usage: tools/cal_record.py [host] [output.jsonl]
  host    compass address, default compass.local
  output  default cal_<timestamp>.jsonl

Start it, then start the calibration on the device (hold the button 2-5 s). It stops
after the calibration quality has been measured (or the fit failed), or after 10 minutes.
Needs: pip install websockets
"""
import asyncio
import json
import sys
import time

import websockets

MAX_SECONDS = 600


async def record(host, path):
    samples = 0
    started = time.time()
    with open(path, "w") as out:
        async with websockets.connect(f"ws://{host}/ws", open_timeout=10, max_size=None) as ws:
            print(f"Connected to {host}, waiting for the calibration...")
            while time.time() - started < MAX_SECONDS:
                try:
                    message = await asyncio.wait_for(ws.recv(), timeout=5)
                except asyncio.TimeoutError:
                    continue
                data = json.loads(message)
                if "calSamples" in data:
                    batch = data["calSamples"]
                    samples += len(batch["s"])
                    out.write(json.dumps({"t": round(time.time() - started, 2), **data}) + "\n")
                    print(f"\r phase {batch['phase']}: {samples} samples", end="", flush=True)
                elif "calEvent" in data:
                    event = data["calEvent"]
                    out.write(json.dumps({"t": round(time.time() - started, 2), **data}) + "\n")
                    print(f"\n{event}")
                    if event["type"] == "quality" or (event["type"] == "fit" and not event["ok"]):
                        break
                out.flush()
    print(f"Saved {samples} samples to {path}")


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "compass.local"
    path = sys.argv[2] if len(sys.argv) > 2 else time.strftime("cal_%Y%m%d_%H%M%S.jsonl")
    asyncio.run(record(host, path))


if __name__ == "__main__":
    main()
