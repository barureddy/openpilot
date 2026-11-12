#!/usr/bin/env python3
"""Inspect local CUSW rlogs; this tool has no CAN transmit or live-device interface."""
import argparse
import bz2
import csv
import json
from collections import Counter
from pathlib import Path


FIELDS = (
  'file', 'mono_ns', 'bus', 'id', 'payload', 'checksum_valid', 'counter',
  'candidate_value', 'candidate_request', 'candidate_later_marker', 'unknown_byte2_high',
  'eps_candidate_early', 'eps_candidate_late',
  'candidate_armed', 'unknown_transition', 'driver_torque_raw', 'motor_torque_raw',
  'lkas_state', 'lkas_fault',
)


def checksum_valid(data: bytes) -> bool:
  crc = 0xFF
  for byte in data[:-1]:
    crc ^= byte
    for _ in range(8):
      crc = ((crc << 1) ^ (0x1D if crc & 0x80 else 0)) & 0xFF
  return bool(data) and data[-1] == (crc ^ 0xFF)


def decode(address: int, data: bytes) -> dict | None:
  """Return observational fields only. Candidate labels are not validated semantics."""
  if address == 0x20C and len(data) == 4:
    return {
      'checksum_valid': checksum_valid(data), 'counter': data[2] & 0xF,
      'candidate_value': ((data[0] << 3) | (data[1] >> 5)) - 1024,
      'candidate_request': (data[1] >> 4) & 1,
      'candidate_later_marker': (data[1] >> 1) & 1,
      'unknown_byte2_high': data[2] >> 4,
      'candidate_armed': (data[2] >> 4) & 1,
      'unknown_transition': (data[2] >> 7) & 1,
    }
  if address == 0x1EC and len(data) == 8:
    return {
      'checksum_valid': checksum_valid(data), 'counter': data[6] & 0xF,
      'eps_candidate_early': (data[3] >> 6) & 1,
      'eps_candidate_late': (data[0] >> 5) & 1,
      'driver_torque_raw': ((data[0] & 0xF) << 8 | data[1]) - 1024,
      'motor_torque_raw': ((data[3] & 0xF) << 8 | data[4]) - 2048,
      'lkas_state': data[2] & 1,
      'lkas_fault': (data[2] >> 1) & 1,
    }
  return None


def events(path: Path):
  # Imports stay local so decoding golden payloads needs only the standard library.
  from openpilot.cereal import log
  data = path.read_bytes()
  if path.suffix == '.zst':
    import zstandard
    with zstandard.ZstdDecompressor().stream_reader(data, read_across_frames=True) as reader:
      data = reader.read()
  elif path.suffix == '.bz2':
    data = bz2.decompress(data)
  yield from log.Event.read_multiple_bytes(data)


def inspect(paths: list[Path], output: Path, bus: int) -> dict:
  counts = Counter()
  with output.open('x', newline='') as stream:
    writer = csv.DictWriter(stream, fieldnames=FIELDS)
    writer.writeheader()
    for path in paths:
      for event in events(path):
        if event.which() != 'can':
          continue
        for frame in event.can:
          # Receipt/rejection flags and sendcan requests are not physical received buses.
          if frame.src != bus:
            continue
          address, data = int(frame.address), bytes(frame.dat)
          if address not in (0x20C, 0x1EC):
            continue
          values = decode(address, data)
          if values is None:
            counts['unexpected_length'] += 1
            continue
          writer.writerow(dict(file=str(path), mono_ns=event.logMonoTime, bus=bus,
                               id=hex(address), payload=data.hex(), **values))
          counts[f'{hex(address)}_frames'] += 1
          counts[f'{hex(address)}_checksum_failures'] += not values['checksum_valid']
  return dict(counts)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('logs', nargs='+', type=Path, help='Local rlog files, including .zst or .bz2')
  parser.add_argument('--output', required=True, type=Path, help='New CSV path; existing files are never overwritten')
  parser.add_argument('--bus', type=int, choices=range(128), default=0, help='Received CAN source (default: 0)')
  args = parser.parse_args()
  for path in args.logs:
    if not path.is_file():
      parser.error(f'Not a local file: {path}')
  print(json.dumps(inspect(args.logs, args.output, args.bus), indent=2))


if __name__ == '__main__':
  main()
