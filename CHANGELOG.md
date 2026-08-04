# Changelog

## 0.1.0

Initial release built around the user's hardware-validated Feetech HLS3950M setup on Raspberry Pi 5:

- Continuous Raspberry Pi RX with CD4051-switched TX.
- Exact transmit-echo removal before SDK status parsing.
- HLS wheel, position, torque-enable, and raw torque/current-target control.
- Non-wrapping signed and absolute software encoder accumulators.
- Regression coverage for observed HLS sign-magnitude boundaries:
  `349 -> -660`, `-31755 -> -4`, and `32745 -> -15`.
- Fast coherent runtime status/snapshot reads.
- Voltage, current, temperature, load, speed, moving state, and fault flags.
- Clearly labeled current-based torque estimate using the HLS3950M's published
  `20.8 kg.cm/A` torque constant.
- Named/raw register access with sign-magnitude decoding.
- JSON configuration backup, ID scanning, baud detection/recovery, and guarded
  EEPROM configuration helpers.
- Repeated zero-speed packets and optional torque release during cleanup.
- README, register map, examples, unit tests, and installable wheel.
