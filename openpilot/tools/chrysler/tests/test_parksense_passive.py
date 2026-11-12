import unittest

from openpilot.tools.chrysler.parksense_passive import checksum_valid, decode


class TestPassiveDecoder(unittest.TestCase):
  def test_recorded_idle_payload(self):
    result = decode(0x20C, bytes.fromhex('80000445'))
    self.assertEqual(result['candidate_value'], 0)
    self.assertEqual(result['candidate_request'], 0)
    self.assertEqual(result['counter'], 4)
    self.assertTrue(result['checksum_valid'])

  def test_recorded_active_payload(self):
    result = decode(0x20C, bytes.fromhex('99d21501'))
    self.assertEqual(result['candidate_value'], 206)
    self.assertEqual(result['candidate_request'], 1)
    self.assertEqual(result['candidate_later_marker'], 1)
    self.assertTrue(result['checksum_valid'])

  def test_corrupted_payload_is_reported(self):
    self.assertFalse(checksum_valid(bytes.fromhex('99d21500')))
    self.assertFalse(decode(0x20C, bytes.fromhex('99d21500'))['checksum_valid'])

  def test_wrong_lengths_and_other_addresses_are_ignored(self):
    self.assertIsNone(decode(0x20C, bytes(8)))
    self.assertIsNone(decode(0x1EC, bytes(4)))
    self.assertIsNone(decode(0x1F6, bytes(4)))


if __name__ == '__main__':
  unittest.main()
