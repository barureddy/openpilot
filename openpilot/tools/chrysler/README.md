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
