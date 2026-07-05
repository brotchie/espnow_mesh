# ESP-NOW Mesh HIL Bench

This example is the hardware-in-the-loop wrapper used for the three analyzer
channels and the optional CYD controller bench. It uses the same app entry point
as the basic examples, but builds with device-specific defaults from `configs/`.

The current analyzer mapping is:

| Device | Role | Target | Serial port | GPIO | Analyzer |
| --- | --- | --- | --- | --- | --- |
| 0 | Controller or satellite | `esp32s3` | `/dev/cu.usbmodem11101` | GPIO5 | CH0 |
| 1 | Satellite | `esp32c3` | `/dev/cu.usbmodem11201` | GPIO6 | CH1 |
| 2 | Satellite | `esp32s3` | `/dev/cu.usbmodem11401` | GPIO5 | CH2 |
| 3 | CYD controller | `esp32` | `/dev/cu.usbserial-1440` | not wired | none |

Build the three-board HIL images:

```sh
idf.py -B build-device1 -DSDKCONFIG=sdkconfig.device1 \
  -DSDKCONFIG_DEFAULTS=configs/device1-satellite-hil.defaults \
  -DIDF_TARGET=esp32c3 -p /dev/cu.usbmodem11201 build flash

idf.py -B build-device2 -DSDKCONFIG=sdkconfig.device2 \
  -DSDKCONFIG_DEFAULTS=configs/device2-satellite-hil.defaults \
  -DIDF_TARGET=esp32s3 -p /dev/cu.usbmodem11401 build flash

idf.py -B build-device0 -DSDKCONFIG=sdkconfig.device0 \
  -DSDKCONFIG_DEFAULTS=configs/device0-controller-hil.defaults \
  -DIDF_TARGET=esp32s3 -p /dev/cu.usbmodem11101 build flash
```

For the CYD-as-controller bench with the display enabled, build
`examples/cyd_controller_visualizer` using
`configs/cyd-controller-hil.defaults` from that example, then flash device 0
with `configs/device0-satellite-hil.defaults` and devices 1/2 with their HIL
satellite defaults.
