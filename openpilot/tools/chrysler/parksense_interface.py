"""Receive-only candidate ParkSense interface. Never grants steering authority.

Freshness windows are offline analysis tolerances, not validated EPS deadlines.
Only valid, in-order received frames update observations. No encoder or transport.
"""
from dataclasses import dataclass
import math

from openpilot.tools.chrysler.parksense_passive import decode


@dataclass(frozen=True)
class Observation:
  phase: str
  issues: tuple[str, ...]
  candidate_value: int | None
  armed: bool | None
  request: bool | None
  eps_early: bool | None
  eps_late: bool | None
  broader_state: int | None
  slow_state: bool | None
  slow_secondary: bool | None


class ParkingObserver:
  """Observe one route and physical bus. Phase names describe correlation only."""

  LENGTHS = {0x20C: 4, 0x1EC: 8, 0x2F2: 8, 0x5E6: 8}

  def __init__(self, bus: int = 0, request_age: float = 0.5, eps_age: float = 0.1):
    if not 0 <= bus < 128:
      raise ValueError('Expected an unflagged received bus')
    if not all(math.isfinite(x) and x > 0 for x in (request_age, eps_age)):
      raise ValueError('Analysis freshness windows must be positive and finite')
    self.bus = bus
    self.ages = {0x20C: request_age, 0x1EC: eps_age, 0x2F2: 0.1, 0x5E6: 0.75}
    self.last_time = -math.inf
    self.samples = {}
    self.counters = {}
    self.errors = {}

  def update(self, time: float, address: int, data: bytes, bus: int = 0, kind: str = 'can') -> bool:
    if kind != 'can' or bus != self.bus or address not in self.LENGTHS:
      return False
    if not math.isfinite(time) or time < self.last_time:
      raise ValueError('Received frames must have finite, nondecreasing timestamps')
    self.last_time = time
    issue = None
    values = None
    if len(data) != self.LENGTHS[address]:
      issue = 'length'
      self.counters.pop(address, None)
    elif address in (0x20C, 0x1EC):
      values = decode(address, data)
      if not values['checksum_valid']:
        issue = 'checksum'
        self.counters.pop(address, None)
      else:
        counter = values['counter']
        previous = self.counters.get(address)
        if previous is not None and counter != (previous + 1) % 16:
          issue = 'counter_discontinuity'
        # Establish a new baseline, but do not use the discontinuous frame as state.
        self.counters[address] = counter
        if address == 0x20C and not values['candidate_request'] and values['candidate_value'] != 0:
          issue = 'nonzero_inactive_candidate'
    elif address == 0x2F2:
      # No checksum/counter definition is asserted for these context messages.
      values = {'broader_state': data[4] & 3}
    else:
      values = {'slow_state': bool(data[2] & 0x20), 'slow_secondary': bool(data[6] & 2)}
    if issue:
      self.samples.pop(address, None)
      self.errors[address] = issue
    else:
      self.samples[address] = (time, values)
      self.errors.pop(address, None)
    return True

  def snapshot(self, time: float) -> Observation:
    if not math.isfinite(time) or time < self.last_time:
      raise ValueError('Snapshot time must be finite and at or after the last frame')
    issues = []

    def current(address):
      if address in self.errors:
        issues.append(f'{address:#x}:{self.errors[address]}')
        return {}
      if address not in self.samples:
        if address in (0x20C, 0x1EC):
          issues.append(f'{address:#x}:missing')
        return {}
      stamp, values = self.samples[address]
      if time - stamp > self.ages[address]:
        if address in (0x20C, 0x1EC):
          issues.append(f'{address:#x}:stale')
        return {}
      return values

    request = current(0x20C)
    eps = current(0x1EC)
    context = current(0x2F2)
    slow = current(0x5E6)
    r = request.get('candidate_request')
    armed = request.get('candidate_armed')
    early, late = eps.get('eps_candidate_early'), eps.get('eps_candidate_late')
    if not request or not eps:
      phase = 'unknown'
    elif r:
      phase = 'request_eps_observed' if early or late else 'request_pending'
    elif early or late:
      phase = 'eps_without_request'
    else:
      phase = 'armed_observed' if armed else 'inactive_observed'
    if eps.get('lkas_fault'):
      issues.append('lkas_fault_observed')
    return Observation(phase, tuple(issues), request.get('candidate_value'), armed, r, early, late,
                       context.get('broader_state'), slow.get('slow_state'), slow.get('slow_secondary'))
