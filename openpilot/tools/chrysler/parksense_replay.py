#!/usr/bin/env python3
"""Replay received parking-interface observations from local logs; never transmits."""
import argparse
from collections import Counter
import csv
from dataclasses import asdict
import json
from pathlib import Path

from openpilot.tools.chrysler.parksense_interface import ParkingObserver
from openpilot.tools.chrysler.parksense_passive import events


def received_frames(paths, bus):
  # Keep only four relevant IDs. Sort across segments and retain equal-time bus order.
  # Startup metadata is not CAN; exact duplicate received frames are removed.
  frames = []
  seen = set()
  for path in paths:
    for event in events(path):
      if event.which() != 'can':
        continue
      for frame in event.can:
        if frame.src != bus or frame.address not in ParkingObserver.LENGTHS:
          continue
        key = (int(event.logMonoTime), int(frame.address), bytes(frame.dat))
        if key not in seen:
          seen.add(key)
          frames.append((*key, str(path)))
  return sorted(frames, key=lambda f: f[0])


def replay(frames, output, bus=0):
  observer = ParkingObserver(bus)
  counts = Counter()
  previous = None
  fields = ['mono_ns', 'file', 'id', 'payload', *asdict(observer.snapshot(0)).keys()]
  with output.open('x', newline='') as stream:
    writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader()
    for ns, address, data, source in frames:
      observer.update(ns / 1e9, address, data, bus)
      snapshot = asdict(observer.snapshot(ns / 1e9))
      counts['received_frames'] += 1
      counts[f'{address:#x}_frames'] += 1
      signature = snapshot['phase']
      if signature != previous:
        counts[f"entries:{snapshot['phase']}"] += 1
        previous = signature
      snapshot['issues'] = ';'.join(snapshot['issues'])
      writer.writerow(dict(mono_ns=ns, file=source, id=hex(address), payload=data.hex(), **snapshot))
  return dict(counts)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('logs', nargs='+', type=Path, help='Segments of ONE route from ONE boot')
  parser.add_argument('--output', required=True, type=Path, help='New observation CSV')
  parser.add_argument('--bus', type=int, choices=range(128), default=0)
  args = parser.parse_args()
  for path in args.logs:
    if not path.is_file():
      parser.error(f'Not a local file: {path}')
  if args.output.exists():
    parser.error('Output already exists')
  frames = received_frames(args.logs, args.bus)
  if not frames:
    parser.error('No matching received parking frames')
  print(json.dumps(replay(frames, args.output, args.bus), indent=2))


if __name__ == '__main__':
  main()
