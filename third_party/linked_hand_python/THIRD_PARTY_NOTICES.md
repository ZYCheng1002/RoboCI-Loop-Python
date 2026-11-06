# Third-party notices

The native USB transport behavior for GCAN/ECAN/Rexon USBCANI-V503
(VID:PID `0c66:000c`) was implemented with reference to the MIT-licensed project:

- `rezaistoni-cloud/Gcan-USB-PRO-II-V503-for-RASPI-ARM64`
- Copyright (c) 2026 Reza Istoni
- License: MIT

The linked_hand implementation is written in C++ and integrated behind the
project's existing `CanTransport` abstraction; the legacy ECanVci transport is
retained as an optional fallback.
