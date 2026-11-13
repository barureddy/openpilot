# CUSW passive parking diagnostics

This branch adds an offline local-log inspector. It does not change steering behavior,
minimum speed, vehicle interfaces, Panda safety, or CAN transmission.

From a configured openpilot development environment:

```sh
python -m openpilot.tools.chrysler.parksense_passive --output parking.csv /path/to/rlog.zst
```

Pass multiple segment files to inspect a full route. Rows retain source filenames and
absolute monotonic timestamps, so repeated startup metadata does not reset the time axis.
The output includes original payloads and checksum results, including failed checksums.
Malformed message lengths are counted and skipped. The tool reads received source 0 by
default; it excludes sendcan and flagged transmission receipts. It cannot establish the
originating ECU from the received bus number. Use a new output filename on each run.

## What the labels mean

`0x20C` is a four-byte candidate parking steering request. In two captured routes it
changes from approximately 5 Hz to 100 Hz during assisted steering. Its candidate
request flag precedes two unnamed EPS status bits by 10–20 ms. An 11-bit field with
an offset of -1024 tracks later EPS motor torque. These are observations, not a
validated command definition, physical unit, or operating envelope.

`0x1EC` contains candidate EPS mode flags at byte 3 bit 6 and byte 0 bit 5. Byte
indices start at zero and bit indices within a byte use LSB=0. The tool deliberately
uses candidate labels rather than adding unverified names to the production DBC.

The source, units, remaining flags, authorization sequence, arbitration, override,
timeout and failure behavior are unresolved. No live control should be inferred from
this decoder. Controlled hardware validation is required before designing actuation
support. Parking-lot space does not establish command or fault behavior.

## Receive-only parking-interface support

`parksense_interface.ParkingObserver` is a reusable, transport-free observer for
candidate 0x20C requests, 0x1EC EPS markers, and 0x2F2/0x5E6 parking context.
It has no command encoder, steering-authority output, live CAN connection or
controller integration. Installing this branch will not enable low-speed steering.

Replay one complete route (one boot) from a configured development environment:

```sh
python -m openpilot.tools.chrysler.parksense_replay --output observations.csv /path/segment0/rlog.zst /path/segment1/rlog.zst
python -m unittest discover -s openpilot/tools/chrysler/tests -v
```

Replay sorts relevant received frames by monotonic timestamp, preserves equal-time
frame order, and removes exact timestamp/address/payload duplicates across supplied
segments. Do not combine routes or boots: monotonic times alone cannot distinguish
such files reliably. Only the four relevant IDs are retained in memory. Full log
segments are decompressed one at a time. Existing output files are never overwritten.

Observations use these labels:

| Phase | Meaning |
|---|---|
| unknown | Missing, stale or invalid request/EPS input |
| inactive_observed | Request, arming and EPS candidate markers clear |
| armed_observed | Arming candidate set, with request and EPS markers clear |
| request_pending | Request candidate set without EPS candidate markers |
| request_eps_observed | Request candidate and at least one EPS marker set |
| eps_without_request | EPS marker remains set without request; may be release transition |

These are observed combinations, not a validated EPS state machine. None means
“safe to steer.” Search state alone never becomes an active-request observation.
EPS early and late markers remain separate fields. Unknown transition flags and
both 0x5E6 state candidates remain separate rather than being treated as a timer or
interchangeable flags. Cruise CANCEL is not interpreted as a ParkSense cancellation.

The observer checks lengths and, for 0x20C/0x1EC, checksums and rolling counters.
A damaged or discontinuous frame invalidates that message's observation. The next
valid sequential frame can restore it; checksum/length failures reset the counter
baseline. Counter gaps may be recording loss, not a vehicle fault. A nonzero
candidate value with the request flag clear is also reported as an inconsistency.
0x2F2/0x5E6 have no verified integrity definition here and are context only.

Default maximum ages are 500 ms for 0x20C, 100 ms for EPS and 0x2F2, and 750 ms
for 0x5E6. These are configurable analysis tolerances for the main request/EPS
streams, accommodating idle cadence; they are **not vehicle watchdog deadlines**.
Call `snapshot(time)` to observe aging even when no new frames arrive. CSV rows
are emitted on received events only; they do not manufacture samples after log end.
Raw torque values have no asserted physical units or validated limits.

Before adding any transmitter, controlled validation must establish message origin,
authorization, physical scaling, operating speed/duration envelope, stock-controller
arbitration, override, cancellation, and communication-failure behavior. The recorded
parking maneuvers do not establish continuous 0–41 mph control.

## Optional on-device receive-only monitor

From the configured openpilot directory on the comma device, while parked:

```sh
python -m openpilot.tools.chrysler.parksense_monitor --duration 600 --output /data/media/0/parksense-monitor.jsonl
```

Use a new filename each time. This manually started process subscribes only to
openpilot's existing received-CAN stream and writes observations at no more than
10 Hz. It exits after ten minutes by default (maximum thirty minutes), or Ctrl-C.
It is not automatically launched, changes no parameters, and publishes no messages.
Normal openpilot and factory parking behavior remain unchanged. Do not operate the
terminal while driving. Keep the original rlogs; the monitor output is only a summary.

A missing/stale stream yields `unknown`, including when no CAN events arrive.
Timestamp errors and invalid events are counted. Diagnostic labels, freshness windows
and candidate torque values are not control permissions, validated watchdogs or limits.
This tool does not make new driving experiments necessary and does not validate active
steering. Device runtime/CPU performance has not been tested on a physical comma unit;
validation here covers simulated receipt, stream loss and recorded-log decoding.
