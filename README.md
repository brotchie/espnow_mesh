# espnow_mesh

`espnow_mesh` is an ESP-IDF component for a controller/satellite ESP-NOW
network. It is built for one controller and many satellites, with application
level packet identity, HMAC authentication, reliable controller fanout,
SoftAP-assisted satellite registration/channel discovery, patch fragmentation,
and monotonic time synchronization.

The repository root is the reusable component. Runnable firmware projects live
under `examples/`.

## Features

- Compile-time controller or satellite role selection.
- Mesh ID and truncated HMAC-SHA256 authentication on application packets.
- Broadcast fanout plus per-satellite ACK tracking and unicast retries with
  exponential backoff.
- Satellite de-duplication using controller boot ID and sequence number.
- SoftAP self-registration so satellites can discover the controller channel
  after boot.
- NTP-style monotonic time samples filtered through a software PLL.
- Optional synchronized GPIO output for skew measurement on a logic analyzer.
- Pull-based patch distribution with block requests, block hashes, full patch
  hash verification, and apply-after-ready coordination.
- HIL test hooks for command delivery, injected packet loss, patch transfer,
  and channel discovery.
- Small status API for controller dashboards and visualizers.

## Install

As a Git dependency in an ESP-IDF app component:

```yaml
dependencies:
  espnow_mesh:
    git: https://github.com/brotchie/espnow_mesh.git
    version: main
```

As a submodule:

```sh
git submodule add https://github.com/brotchie/espnow_mesh.git components/espnow_mesh
```

Then add the component to your app component requirements:

```cmake
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES espnow_mesh
)
```

Start the mesh from your app:

```c
#include "esp_err.h"
#include "espnow_mesh.h"

void app_main(void)
{
    ESP_ERROR_CHECK(espnow_mesh_start(NULL));
}
```

## Examples

Activate ESP-IDF first:

```sh
. ~/esp/esp-idf/export.sh
```

Build the basic controller:

```sh
cd examples/basic_controller
idf.py -B build -DIDF_TARGET=esp32s3 build
```

Build the basic satellite:

```sh
cd examples/basic_satellite
idf.py -B build -DIDF_TARGET=esp32c3 build
```

Build the 4 inch CYD controller visualizer:

```sh
cd examples/cyd_controller_visualizer
idf.py -B build -DIDF_TARGET=esp32 build
```

Build the CYD controller visualizer with HIL controller behavior:

```sh
cd examples/cyd_controller_visualizer
idf.py -B build-hil -DSDKCONFIG=sdkconfig.hil \
  -DSDKCONFIG_DEFAULTS=configs/cyd-controller-hil.defaults \
  -DIDF_TARGET=esp32 build
```

The CYD dependency is only declared by `examples/cyd_controller_visualizer`.
The core `espnow_mesh` component does not depend on LVGL or display drivers.

The HIL profiles and bench notes are in `examples/hil_bench` and
`docs/hil-testbench.md`.

## Public API

- `espnow_mesh_start()` starts the mesh engine in its own FreeRTOS task.
- `espnow_mesh_run()` runs the mesh engine on the current task and blocks.
- `espnow_mesh_get_time_us()` returns the controller monotonic time estimate.
- `espnow_mesh_is_time_synced()` reports whether the local time estimate is
  valid.
- `espnow_mesh_get_status()` returns controller/satellite status for UIs.
- `espnow_mesh_role()` reports the compiled controller/satellite role.

## Configuration

Run `idf.py menuconfig` and open `ESP-NOW mesh component`.

Important options:

- `CONFIG_ESPNOW_MESH_ROLE_CONTROLLER` /
  `CONFIG_ESPNOW_MESH_ROLE_SATELLITE`: build-time role switch.
- `CONFIG_ESPNOW_MESH_CHANNEL`: default ESP-NOW and registration AP channel.
- `CONFIG_ESPNOW_MESH_ID`, `CONFIG_ESPNOW_MESH_AUTH_ENABLE`, and
  `CONFIG_ESPNOW_MESH_AUTH_KEY`: application packet identity and
  authentication. Change the development key before deployment.
- `CONFIG_ESPNOW_MESH_SEND_RETRIES`,
  `CONFIG_ESPNOW_MESH_RELIABLE_MAX_ATTEMPTS`,
  `CONFIG_ESPNOW_MESH_RELIABLE_INITIAL_BACKOFF_MS`,
  `CONFIG_ESPNOW_MESH_RELIABLE_MAX_BACKOFF_MS`,
  `CONFIG_ESPNOW_MESH_RELIABLE_RETRY_JITTER_MS`, and
  `CONFIG_ESPNOW_MESH_RELIABLE_DEADLINE_MS`: reliable fanout behavior.
- `CONFIG_ESPNOW_MESH_TIME_SYNC_INTERVAL_MS`,
  `CONFIG_ESPNOW_MESH_TIME_SYNC_MAX_DELAY_US`, and the
  `CONFIG_ESPNOW_MESH_TIME_SYNC_PLL_*` settings: monotonic time sync cadence,
  PLL gains, frequency clamp, and outlier rejection.
- `CONFIG_ESPNOW_MESH_REGISTRATION_AP_ENABLE` and
  `CONFIG_ESPNOW_MESH_REGISTRATION_*`: SoftAP self-registration and channel
  discovery behavior.
- `CONFIG_ESPNOW_MESH_SYNC_OUTPUT_*`: logic-analyzer GPIO output.
- `CONFIG_ESPNOW_MESH_HIL_*`: deterministic HIL test mode, injected loss, and
  patch-distribution test settings.

## Runtime Model

The controller broadcasts critical data packets, tracks every known satellite
expected to ACK the active sequence, then retries missing satellites with
unicast packets. Satellites process the first copy of a sequence and ACK every
valid copy so controller retries can converge.

Satellites periodically send time-sync requests. The controller replies with
controller receive/send timestamps. Satellites estimate controller monotonic
time from the four timestamps and feed accepted samples into the PLL. This is
not UTC wall-clock time; it is a shared monotonic time base.

When registration is enabled, the controller also runs a SoftAP. A satellite
that does not hear controller ESP-NOW traffic scans for the registration SSID,
briefly associates, adopts the AP channel, and returns to ESP-NOW. The
controller records the station MAC from the association event and includes that
satellite in future ACK quorums.

Patch distribution uses a pull model. The controller offers patch metadata.
Satellites request missing blocks one at a time, verify block hashes and the
full patch hash, report ready, and apply only after the controller sends an
apply command.

This is intentionally a star topology around one controller. It does not route
through satellites.
