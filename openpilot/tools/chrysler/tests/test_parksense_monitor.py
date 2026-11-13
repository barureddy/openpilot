import io
import json
from types import SimpleNamespace
import unittest

from openpilot.tools.chrysler.parksense_monitor import monitor
from openpilot.tools.chrysler.tests.test_parksense_interface import frame


class TestMonitor(unittest.TestCase):
  def run_session(self, first, duration=.8):
    now = [0.0]
    calls = [0]
    def receive():
      now[0] += .05
      calls[0] += 1
      return first if calls[0] == 1 else None
    out = io.StringIO()
    counts = monitor(receive, out, duration, clock=lambda: now[0])
    return [json.loads(row) for row in out.getvalue().splitlines()], counts

  def event(self, kind='can', valid=True, stamp=0, bus=0):
    return SimpleNamespace(which=lambda: kind, valid=valid, logMonoTime=stamp,
                           can=[SimpleNamespace(address=a, dat=frame(a, request=True, early=True), src=bus)
                                for a in [0x20C, 0x1EC]])

  def test_observed_active_becomes_unknown_when_stream_stops(self):
    rows, counts = self.run_session(self.event())
    self.assertEqual(rows[0]['phase'], 'request_eps_observed')
    self.assertEqual(rows[-1]['phase'], 'unknown')
    self.assertIn('0x20c:stale', rows[-1]['issues'])
    self.assertEqual(counts['candidate_frames'], 2)
    self.assertLessEqual(len(rows), 9)
    self.assertLessEqual(rows[-1]['elapsed'], .9)

  def test_invalid_direction_source_and_timestamp_never_activate(self):
    for event in [self.event(kind='sendcan'), self.event(valid=False), self.event(bus=128), self.event(stamp=10**10)]:
      rows, counts = self.run_session(event)
      self.assertTrue(all(row['phase'] == 'unknown' for row in rows))
      self.assertNotIn('candidate_frames', counts)

  def test_duration_is_bounded(self):
    for duration in [0, -1, 1801, float('nan')]:
      with self.assertRaises(ValueError):
        monitor(lambda: None, io.StringIO(), duration)


if __name__ == '__main__':
  unittest.main()
