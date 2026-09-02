# ESP32 BikeControl — ESP-IDF

Minimal OpenBikeControl controller for ESP32-C6-DevKitC-1.

## Controls

- BOOT < 2 s -> SHIFT DOWN (0x02)
- BOOT > 3 s -> SHIFT UP (0x01)
- 2..3 s -> no action

A long press sends SHIFT UP once at 3 seconds.

## BLE

OpenBikeControl service:
`d273f680-d548-419d-b9d1-fa0472345229`

Button State:
`d273f681-d548-419d-b9d1-fa0472345229`

Haptic:
`d273f682-d548-419d-b9d1-fa0472345229`

App Information:
`d273f683-d548-419d-b9d1-fa0472345229`

Button State packets:

Shift Up:
`01 01 01` pressed
`01 01 00` released

Shift Down:
`01 02 01` pressed
`01 02 00` released

## Build

PlatformIO:
`pio run`

Upload:
`pio run -t upload`

Monitor:
`pio device monitor`

The project uses native ESP-IDF NimBLE, not Arduino and not NimBLE-Arduino.
