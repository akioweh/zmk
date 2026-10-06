# Experimental Native PTP Core

This fork ports the HID interface from [Pete Johanson's PTP prototype](https://github.com/petejohanson/zmk/tree/455ad3eafaccf6f85be23c45fae6d74f113e098c) to the existing Zephyr 4.1 ZMK baseline.
It exposes a separate USB HID interface and Bluetooth HID service using the same descriptor and report encoder.
The target is native Linux touchpad input, not firmware-side gestures or Windows certification.

Sensor adapters belong in external ZMK modules and use the public producer API; this fork contains only the generic PTP infrastructure. Split contact forwarding uses the existing BLE and wired transports. Existing mouse input listeners are unchanged; remove any legacy listener/driver for the same touchpad to avoid duplicate pointer events.

## Configuration

Set `CONFIG_ZMK_TRACKPAD=y` on the central/unibody firmware, and on any peripheral supplying contacts. Use matching finger counts and logical coordinate ranges on both halves. Configure:

- `CONFIG_ZMK_TRACKPAD_FINGERS`: 3–5 concurrent slots, default 5.
- `CONFIG_ZMK_TRACKPAD_LOGICAL_X/Y`: maximum sensor coordinates (origin at the top left).
- `CONFIG_ZMK_TRACKPAD_PHYSICAL_X/Y`: actual active surface size in 0.1 mm units. Defaults are placeholders, not calibration.
- `CONFIG_ZMK_TRACKPAD_PAD_TYPE`: 0 clickpad, 1 pressure pad, 2 external buttons only (default).
- `CONFIG_ZMK_BLE_PTP_REPORT_QUEUE_SIZE`: complete host reports per Bluetooth profile, default 8.
- `CONFIG_ZMK_TRACKPAD_SPLIT_QUEUE_SIZE`: complete split frames per FIFO, default 8.
- `CONFIG_ZMK_TRACKPAD_SPLIT_SOURCE`: central-side split source ID supplying the logical touchpad, default 0. One logical touchpad is supported; do not mix local and remote producers. Physical dimensions and pad type are host-side settings only.

USB needs two HID interfaces and an interrupt packet large enough for the report; defaults are adjusted when enabled and incompatible overrides fail compilation. Keyboard boot protocol and existing mouse reporting remain separate.

At five contacts the USB report is 29 bytes; the Bluetooth value excludes its Report ID and is 28 bytes. BLE requires a negotiated ATT MTU of at least 31. The backend returns `-EMSGSIZE` until that is available. Three-contact reports fit the minimum ATT MTU of 23.

## Driver Interface

Include `zmk/ptp.h` when `CONFIG_ZMK_TRACKPAD` is enabled. From thread context, submit a complete active-contact snapshot:

```c
struct zmk_ptp_frame frame = {
    .scan_time = zmk_ptp_scan_time(),
    .contact_count = 1,
    .contacts = {{.id = 0, .x = 100, .y = 200, .confidence = true}},
};
int err = zmk_ptp_submit_frame(&frame);
```

- IDs are stable slots in `[0, CONFIG_ZMK_TRACKPAD_FINGERS)`. Do not reuse a slot until an accepted snapshot omits its previous contact.
- Missing contacts produce explicit tip-up records at their last accepted coordinates, once. An empty snapshot lifts all fingers. Confidence stays false once a contact is classified as unintentional.
- All contacts in a frame share its wrapping, 16-bit scan time in 100 µs units. Synthetic cleanup retains the last accepted source scan time; it does not substitute the central's clock.
- Button bits are integrated, external primary and external secondary. Do not synthesize taps/scrolling here; the Linux input stack supplies those gestures.
- A zero return means the local host/split transport accepted the complete frame, not that the central or host received it. Retry negative errors with a complete snapshot, preserving contact lifecycles. ISR calls are rejected.
- USB failures do not advance accepted state. BLE backpressure never overwrites queued frames; notifications retry resource failures from system-workqueue context. Retained connection references prevent replay to another peer/session.
- `zmk_ptp_release()` requests cleanup, with deferred retry for transport failures. Retry lock-contention errors yourself. Endpoint changes release the old destination and do not route that release to the newly selected profile. A suspended old BLE host cannot block another profile's queue.

## Split forwarding

The producer uses the same `zmk_ptp_submit_frame()` and `zmk_ptp_release()` API on either half. No sensor-specific transport, scalar input listener or second split stack is involved:

`producer → complete frame → existing split event routing → central core → USB/BLE host`

A new contact-frame event extends `zmk_split_transport_peripheral_event`. Wired forwarding reuses its envelope, CRC and UART backends; the receiver validates typed payload lengths. BLE adds an encrypted notification characteristic to the existing split service. Each snapshot is one 30-byte value, independent of the configured 3–5 fingers: little-endian sequence/scan time, count/buttons, and five compact ID/confidence/X/Y records. No host HID report or host state is carried across the split.

Split BLE needs ATT MTU **33 or greater**, including with three fingers. The central requests MTU negotiation automatically; sender admission returns `-EMSGSIZE` until ready. Notifications retain their original connection references and retry resource pressure, rather than replaying old queued frames into a new session. Both halves must run contact-capable firmware.

Snapshots are copied, FIFO-queued and applied atomically. The core checks the bridge epoch under its endpoint lock before admitting a report; invalidated handoffs are rejected. Transfers already admitted cannot be withdrawn. A 16-bit sequence detects missed frames and slot reuse across a lost lift. Queue overflow discards whole frames, cancels the previous lifetime with unconfident lifts, and recovers from the latest complete snapshot. Gaps/overflow can lose gestures; this is not an end-to-end reliable/acknowledged delivery protocol.

A peripheral caches its latest validated **physical observation**, even if admission fails. Identical retries keep their sequence; new observations (including changed scan time or an offline lift) advance it. Active snapshots send 50 ms heartbeats with the same sequence and timestamp; the central refreshes the lease without duplicate HID reports. Empty snapshots have two heartbeat attempts, then idle traffic stops. Disconnect, transport/host selection changes, or 300 ms without valid frames invalidate pending snapshots and cancel the old contacts/buttons. These are transport-health heartbeats, not sensor-health checks: a producer must call `zmk_ptp_release()` when it stops or loses its sensor.

Thread contention still returns `-EAGAIN`; producers must handle that. Buffered forwarding retries host backpressure without partial frames. The lease/heartbeat timings need physical validation on slower transports; no throughput or Linux cancellation behavior is claimed from compilation.

The host can read capabilities, an all-zero uncertified status blob, native input mode (3), and per-endpoint selective reporting. Surface/button switches are honored. Mouse input mode (0) is explicitly unsupported. USB reset/disconnection and BLE disconnection reset host state; BLE's HID control point honors suspend/resume. Cleanup reports do not wake an inactive USB host.

Reporting semantics follow Microsoft's [PTP collection](https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/touchpad-windows-precision-touchpad-collection) and [configuration collection](https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/touchpad-configuration-collection). This is not a claim of certified Windows support.

## Checks

From an existing West workspace, replace `<zmk>` with the source checkout/worktree path:

```sh
ZEPHYR_TOOLCHAIN_VARIANT=host west build <zmk>/app/tests/ptp \
  -d build/ptp-tests -b native_sim/native/64
build/ptp-tests/zephyr/zephyr.exe

ZEPHYR_TOOLCHAIN_VARIANT=host west build <zmk>/app/tests/ptp \
  -d build/ptp-peripheral-tests -b native_sim/native/64 -- \
  -DEXTRA_CONF_FILE=peripheral.conf
build/ptp-peripheral-tests/zephyr/zephyr.exe
```

Sensor decoding and driver tests belong to their respective external modules.

These tests exercise real core state/encoding/workqueue behavior with mocked USB/BLE admission. They check contact lifecycles, descriptor lengths/units, malformed frames, backpressure, endpoint routing, selective reporting, suspend/reset and ISR rejection, plus split encoding, sequence wrap/gaps, heartbeat/lease expiry, overflow recovery, offline lifts and fragmented/corrupt wired framing. They do not simulate radio delivery or host recognition.

For a compile-only firmware check, add `-DEXTRA_CONF_FILE=<zmk>/app/tests/ptp/firmware.conf` to a central build, or `-DEXTRA_CONF_FILE=<zmk>/app/tests/ptp/peripheral-firmware.conf` to a peripheral build. Its dimensions are synthetic; do not use them as sensor calibration. PTP remains opt-in at the ZMK level; board/config repositories can enable the driver.

Physical USB/Bluetooth enumeration, libinput behavior, radio throughput and disconnect/suspend races still require hardware validation with the sensor and both halves connected.
