import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
from types import SimpleNamespace

from openpilot.tools.chrysler.parksense_interface import ParkingObserver
from openpilot.tools.chrysler.parksense_replay import received_frames, replay


def frame(address, counter=0, request=False, armed=False, early=False, late=False):
  # Test fixtures only; no encoder is provided by the receive-only interface.
  data = bytearray.fromhex('80000000' if address == 0x20C else '0400000800000000')
  if address == 0x20C:
    data[1] |= int(request) << 4
    data[2] = counter | (int(armed) << 4)
  else:
    data[0] |= int(late) << 5
    data[3] |= int(early) << 6
    data[6] = counter
  crc = 255
  for byte in data[:-1]:
    crc ^= byte
    for _ in range(8):
      crc = ((crc << 1) ^ (0x1D if crc & 128 else 0)) & 255
  data[-1] = crc ^ 255
  return bytes(data)


class TestParkingObserver(unittest.TestCase):
  def test_search_request_eps_and_shutdown_sequence(self):
    o = ParkingObserver()
    o.update(0, 0x20C, frame(0x20C))
    self.assertEqual(o.snapshot(0).phase, 'unknown')
    o.update(0, 0x1EC, frame(0x1EC))
    self.assertEqual(o.snapshot(0).phase, 'inactive_observed')
    o.update(.01, 0x20C, frame(0x20C, 1, armed=True))
    self.assertEqual(o.snapshot(.01).phase, 'armed_observed')
    o.update(.02, 0x20C, frame(0x20C, 2, request=True, armed=True))
    self.assertEqual(o.snapshot(.02).phase, 'request_pending')
    o.update(.03, 0x1EC, frame(0x1EC, 1, early=True))
    self.assertEqual(o.snapshot(.03).phase, 'request_eps_observed')
    o.update(.04, 0x20C, frame(0x20C, 3))
    self.assertEqual(o.snapshot(.04).phase, 'eps_without_request')
    o.update(.05, 0x1EC, frame(0x1EC, 2))
    self.assertEqual(o.snapshot(.05).phase, 'inactive_observed')

  def test_missing_stale_bad_crc_and_recovery(self):
    o = ParkingObserver()
    o.update(0, 0x20C, frame(0x20C, request=True))
    o.update(0, 0x1EC, frame(0x1EC, early=True))
    self.assertEqual(o.snapshot(.11).phase, 'unknown')
    self.assertIn('0x1ec:stale', o.snapshot(.11).issues)
    broken = bytearray(frame(0x1EC, 1, early=True))
    broken[-1] ^= 1
    o.update(.12, 0x1EC, bytes(broken))
    self.assertIsNone(o.snapshot(.12).eps_early)
    self.assertIn('0x1ec:checksum', o.snapshot(.12).issues)
    o.update(.13, 0x1EC, frame(0x1EC, 2, early=True))
    self.assertEqual(o.snapshot(.13).phase, 'request_eps_observed')
    self.assertEqual(o.snapshot(.51).phase, 'unknown')
    self.assertIn('0x20c:stale', o.snapshot(.51).issues)

  def test_counter_wrap_jump_and_duplicate(self):
    o = ParkingObserver()
    o.update(0, 0x20C, frame(0x20C, 15))
    o.update(.01, 0x20C, frame(0x20C, 0))
    self.assertNotIn('0x20c:counter_discontinuity', o.snapshot(.01).issues)
    for t, c in [(.02, 2), (.03, 2)]:
      o.update(t, 0x20C, frame(0x20C, c))
      self.assertIn('0x20c:counter_discontinuity', o.snapshot(t).issues)
      self.assertIsNone(o.snapshot(t).request)
    o.update(.04, 0x20C, frame(0x20C, 3))
    self.assertEqual(o.snapshot(.04).request, 0)

  def test_reject_wrong_source_direction_and_time(self):
    o = ParkingObserver()
    for bus, kind in [(128, 'can'), (0, 'sendcan'), (2, 'can')]:
      self.assertFalse(o.update(1, 0x20C, frame(0x20C), bus, kind))
    o.update(2, 0x20C, frame(0x20C))
    for t in [1, float('nan'), float('inf')]:
      with self.assertRaises(ValueError):
        o.update(t, 0x20C, frame(0x20C))
    with self.assertRaises(ValueError):
      o.snapshot(1)
    o.update(2, 0x1EC, b'bad')
    self.assertIn('0x1ec:length', o.snapshot(2).issues)

  def test_context_does_not_grant_request_and_flags_are_independent(self):
    o = ParkingObserver()
    o.update(0, 0x20C, frame(0x20C))
    o.update(0, 0x1EC, frame(0x1EC))
    o.update(0, 0x2F2, bytes.fromhex('0000000001000000'))
    o.update(0, 0x5E6, bytes.fromhex('0000200000000000'))
    s = o.snapshot(0)
    self.assertEqual(s.phase, 'inactive_observed')
    self.assertTrue(s.slow_state)
    self.assertFalse(s.slow_secondary)
    self.assertEqual(s.broader_state, 1)
    self.assertIsNone(o.snapshot(1).slow_state)

  def test_replay_sorts_deduplicates_and_never_overwrites(self):
    def event(ns, src=0, kind='can'):
      return SimpleNamespace(logMonoTime=ns, which=lambda: kind,
                             can=[SimpleNamespace(src=src, address=0x20C, dat=frame(0x20C))])
    with patch('openpilot.tools.chrysler.parksense_replay.events', return_value=iter([
      event(2), event(1), event(1), event(3, 128), event(4, kind='sendcan')])):
      frames = received_frames([Path('fake')], 0)
    self.assertEqual([f[0] for f in frames], [1, 2])
    with TemporaryDirectory() as directory:
      out = Path(directory) / 'observations.csv'
      self.assertEqual(replay(frames, out)['received_frames'], 2)
      with self.assertRaises(FileExistsError):
        replay(frames, out)

  def test_cruise_cancel_is_not_a_parking_state_input(self):
    o = ParkingObserver()
    o.update(0, 0x20C, frame(0x20C, request=True))
    o.update(0, 0x1EC, frame(0x1EC, early=True))
    self.assertFalse(o.update(.01, 0x2FA, bytes.fromhex('0100000000000000')))
    self.assertEqual(o.snapshot(.01).phase, 'request_eps_observed')

  def test_invalid_analysis_configuration(self):
    for args in [{'bus': 128}, {'eps_age': 0}, {'request_age': float('nan')}]:
      with self.assertRaises(ValueError):
        ParkingObserver(**args)


if __name__ == '__main__':
  unittest.main()
