# larpix-control

Control software for LArPix ASIC systems, with PACMAN transport support.

[![Documentation Status](https://readthedocs.org/projects/larpix-control/badge/?version=stable)](https://larpix-control.readthedocs.io/en/stable/?badge=stable)
![Build Status](https://github.com/larpix/larpix-control/actions/workflows/run-tests.yml/badge.svg)

---

## Release Notes (current working branch)

This update focuses on compatibility with newer PACMAN firmware message formats, especially widened timestamp handling.

### Highlights

- Added a new low-level PACMAN message codec in `larpix/format/message.py`.
  - Uses a 24-byte header and 24-byte word format.
  - Uses 64-bit timestamp/payload fields in word/header packing.
  - Adds explicit uint64 validation helpers for timestamp and payload fields.

- Updated `larpix/format/pacman_msg_format.py` to operate as a compatibility translator.
  - Preserves the historic public API (`format_header`, `parse_header`, `format_word`, `parse_word`, `format_msg`, `parse_msg`, `format`, `parse`).
  - Internally maps to/from the new wire protocol implementation in `larpix/format/message.py`.
  - Converts 8-byte packet payload bytes to/from uint64 payload words.
  - Preserves legacy behavior such as `REP + PING -> PONG`.

- Updated HDF5 packet schema for widened receipt timestamps.
  - `receipt_timestamp` is now stored as `u8` where applicable in `larpix/format/hdf5format.py`.

- Updated PACMAN/V3 register and default configuration values to match current firmware/register definitions.
  - Includes updates in `larpix/configuration/configuration_v3.py`, `larpix/configs/chip/default_v3.json`, and `larpix/io/pacman_io.py`.

### Why this release matters

Older assumptions around PACMAN message size and timestamp width can truncate or misinterpret data when used with newer firmware. This release updates the serialization path and compatibility layer so the existing `larpix-control` API continues to work while carrying full-width values end-to-end.

### Notes for users

- If you parse PACMAN messages outside of `larpix-control`, verify your decoder matches the current header/word layout.
- If you consume HDF5 output, ensure downstream readers handle `receipt_timestamp` as 64-bit (`u8`).
- If you depend on specific V3/PACMAN register defaults, review updated config files before deployment.

---

## Installation

Python 3.9+ is supported.

```bash
pip install larpix-control
```

To install from source:

```bash
pip install .
```

To uninstall:

```bash
pip uninstall larpix-control
```

---

## Minimal example

```python
from larpix import Controller, Packet_v2
from larpix.io import FakeIO
from larpix.logger import StdoutLogger

controller = Controller()
controller.io = FakeIO()
controller.logger = StdoutLogger(buffer_length=0)
controller.logger.enable()

chip = controller.add_chip('1-1-2', version=2)
chip.config.threshold_global = 25
controller.write_configuration('1-1-2', chip.config.register_map['threshold_global'])

packet = Packet_v2(b'\x02\x91\x15\xcd[\x07\x85\x00')
controller.io.queue.append(([packet], packet.bytes()))
controller.run(0.05, 'test run')
```

---

## Testing

After installation, run tests from repository root:

```bash
pytest
```

---

## Documentation

Full docs and API reference are available on ReadTheDocs:

- https://larpix-control.readthedocs.io/en/stable/

