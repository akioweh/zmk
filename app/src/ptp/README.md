# Experimental Native PTP Core

This fork ports the HID interface from [Pete Johanson's PTP prototype](https://github.com/petejohanson/zmk/tree/455ad3eafaccf6f85be23c45fae6d74f113e098c) to the existing Zephyr 4.1 ZMK baseline.
It exposes a separate USB HID interface and Bluetooth HID service using the same descriptor and report encoder.
The target is native Linux touchpad input, not firmware-side gestures or Windows certification.

**There is no raw-contact sensor adapter or split contact transport yet.** Existing mouse input listeners are unchanged. Enabling this feature alone does not make an Azoteq trackpad multitouch.

## Configuration

Set `CONFIG_ZMK_TRACKPAD=y` on the central/unibody firmware. Configure:

- `CONFIG_ZMK_TRACKPAD_FINGERS`: 3–5 concurrent slots, default 5.
- `CONFIG_ZMK_TRACKPAD_LOGICAL_X/Y`: maximum sensor coordinates (origin at the top left).
- `CONFIG_ZMK_TRACKPAD_PHYSICAL_X/Y`: actual active surface size in 0.1 mm units. Defaults are placeholders, not calibration.
- `CONFIG_ZMK_TRACKPAD_PAD_TYPE`: 0 clickpad, 1 pressure pad, 2 external buttons only (default).
- `CONFIG_ZMK_BLE_PTP_REPORT_QUEUE_SIZE`: complete reports per Bluetooth profile, default 8.

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
- All contacts in a frame share its wrapping, 16-bit scan time in 100 µs units.
- Button bits are integrated, external primary and external secondary. Do not synthesize taps/scrolling here; the Linux input stack supplies those gestures.
- A zero return means USB accepted the packet or BLE accepted the complete report into its bounded queue, not that the host received it. Retry negative errors with a complete snapshot, preserving contact lifecycles. ISR calls are rejected.
- USB failures do not advance accepted state. BLE backpressure never overwrites queued frames; notifications retry resource failures from system-workqueue context. Retained connection references prevent replay to another peer/session.
- `zmk_ptp_release()` requests cleanup, with deferred retry for transport failures. Retry lock-contention errors yourself. Endpoint changes release the old destination and do not route that release to the newly selected profile. A suspended old BLE host cannot block another profile's queue.

The host can read capabilities, an all-zero uncertified status blob, native input mode (3), and per-endpoint selective reporting. Surface/button switches are honored. Mouse input mode (0) is explicitly unsupported. USB reset/disconnection and BLE disconnection reset host state; BLE's HID control point honors suspend/resume. Cleanup reports do not wake an inactive USB host.

Reporting semantics follow Microsoft's [PTP collection](https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/touchpad-windows-precision-touchpad-collection) and [configuration collection](https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/touchpad-configuration-collection). This is not a claim of certified Windows support.

## Checks

From an existing West workspace, replace `<zmk>` with the source checkout/worktree path:

```sh
ZEPHYR_TOOLCHAIN_VARIANT=host west build <zmk>/app/tests/ptp \
  -d build/ptp-tests -b native_sim/native/64
build/ptp-tests/zephyr/zephyr.exe
```

These tests exercise real core state/encoding/workqueue behavior with mocked USB/BLE admission. They check contact lifecycles, descriptor lengths/units, malformed frames, backpressure, endpoint routing, selective reporting, suspend/reset and ISR rejection. They do not simulate radio delivery or host recognition.

For a compile-only firmware check, add `-DEXTRA_CONF_FILE=<zmk>/app/tests/ptp/firmware.conf` to a central build. Its dimensions are synthetic; do not use them as sensor calibration. Normal firmware leaves PTP disabled.

Physical USB/Bluetooth enumeration, libinput behavior, radio throughput and disconnect/suspend races still require hardware validation once a contact producer is connected.
