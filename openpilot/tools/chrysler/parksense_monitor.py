#!/usr/bin/env python3
"""Manually started, receive-only ParkSense monitor for a configured comma device."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import time

from openpilot.tools.chrysler.parksense_interface import ParkingObserver


def monitor(receive, stream, duration, clock=time.monotonic, bus=0):
  """Receive CAN events and write bounded-rate diagnostic snapshots, never commands."""
  if not 0 < duration <= 1800:
    raise ValueError('Duration must be between 0 and 1800 seconds')
  observer = ParkingObserver(bus)
  counts = Counter()
  start = clock()
  next_sample = start
  next_flush = start
  try:
    while clock() - start < duration:
      event = receive()  # Production subscriber has a 100 ms timeout.
      now = clock()
      if event is not None:
        if event.which() != 'can':
          counts['ignored_non_can'] += 1
        elif not event.valid:
          counts['invalid_can_events'] += 1
        else:
          stamp = event.logMonoTime / 1e9
          if stamp > now or stamp < observer.last_time:
            counts['timestamp_errors'] += 1
          else:
            for frame in event.can:
              if observer.update(stamp, int(frame.address), bytes(frame.dat), int(frame.src)):
                counts['candidate_frames'] += 1
      if now >= next_sample:
        row = dict(mono=now, elapsed=now-start, **asdict(observer.snapshot(now)), diagnostics=dict(counts))
        stream.write(json.dumps(row) + '\n')
        counts['snapshots'] += 1
        next_sample = now + 0.1
      if now >= next_flush:
        stream.flush()
        next_flush = now + 1.0
  except KeyboardInterrupt:
    counts['interrupted'] += 1
  stream.flush()
  return dict(counts)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--output', required=True, type=Path, help='New local JSONL file; never overwrites')
  parser.add_argument('--duration', type=int, default=600, choices=range(1, 1801), metavar='SECONDS')
  parser.add_argument('--bus', type=int, default=0, choices=range(128))
  args = parser.parse_args()
  if args.output.exists():
    parser.error('Output already exists; choose a new filename')
  # Import only in live entry point; unit tests need no device or IPC bindings.
  from openpilot.cereal import messaging
  with args.output.open('x') as stream:
    sock = messaging.sub_sock('can', conflate=False, timeout=100)
    print('Receive-only monitor started. Steering behavior is unchanged. Ctrl-C stops recording.', flush=True)
    counts = monitor(lambda: messaging.recv_one(sock), stream, args.duration, bus=args.bus)
  print(json.dumps(counts, indent=2))


if __name__ == '__main__':
  main()
