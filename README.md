# zmk-esb-endpoint

Turns the last BLE profile slot on a ZMK keyboard into an ESB PTX endpoint
that talks to a matching USB dongle. Pick that profile and ESB takes the
radio; pick a different one and BLE comes back.

## Use

In your `west.yml`:

```yaml
- name: zmk-esb-endpoint
  remote: efogdev
  revision: "main"
  submodules: true
```

Enable it for the keyboard:
```
CONFIG_ZMK_ESB_ENDPOINT=y
```

Devicetree needs a single `zmk,esb-endpoint` node with the pairing/data
base addresses, prefixes and RF channel. See `dts/bindings/`.

It auto-activates when the user selects the last BLE profile
(`ZMK_BLE_PROFILE_COUNT - 1`). First time around, the keyboard broadcasts
PAIR beacons; press the pair button on the dongle to bind. The binding is
then remembered in settings so subsequent boots go straight to CONNECTED.

## Dongle 

You bring your own PRX. The wire protocol is in `include/zmk_esb/protocol.h` —
five message types total (BEACON / PAIR_REQ / PAIR_RESP / HID_REPORT /
DISCONNECT). HID report bodies are ZMK's `zmk_hid_*_report_body` structs
copied verbatim into an ESB payload, so the dongle can hand them straight
to USB HID with nothing more than the report-id prefix byte.

Mouse reports are sent with `noack=1` — a dropped pointer frame is self
correcting. Keyboard and consumer reports are ACKed; a lost release packet
will strand a key on the host.

## Configuration

| Kconfig                                        | What it does |
|------------------------------------------------|--------------|
| `ZMK_ESB_ENDPOINT_HID_NOACK`                   | Mouse fire-and-forget (default on). |
| `ZMK_ESB_ENDPOINT_RETRANSMIT_COUNT` / `_DELAY_US` | ACKed-packet retransmit policy. |
| `ZMK_ESB_ENDPOINT_BEACON_INTERVAL_MS`          | Beacon rate while unpaired. |
| `ZMK_ESB_ENDPOINT_BLE_QUIESCE_MS`              | Quiet time after adv-stop before radio swap. |
| `ZMK_ESB_ENDPOINT_BOOT_CHECK_DELAY_MS`         | Settings-load delay before polling the boot profile. |

## SoC note

**Written and tuned for nRF52833.** Most of the ugly bits are SoC-specific
and the module will not just build and run on anything else without edits:

- `BT_LL_PPI_MASK` in `esb_transport.c` clears the PPI channels the BLE LL
  owns while ESB holds the radio. The mask (channels 6–19 + 22/23/25) is
  based on what the `BT_LL_SW_SPLIT` controller uses on nRF52. If you're on
  a different NCS/Zephyr version, confirm the channels before trusting it.
- The RADIO IRQ is swapped by patching `VTOR[16+RADIO_IRQn]` in a RAM copy
  of the vector table. `NVIC_NUM_VECTORS` is 64 (16 system + 48 external on
  the 52833) — wrong on any SoC with more external interrupts.
- `TIMER2` is taken for the ESB system timer.
- HFXO is held for the lifetime of the ESB slot so the LFRC calibrator
  doesn't stop HFCLK out from under an in-flight packet.

**nRF52840 should work with modifications.** Start by:

- Reviewing the PPI mask against your LL build.
- Checking `NVIC_NUM_VECTORS` — the 52840 is also 48 external so this
  happens to match, but verify.
- Making sure no other subsystem on your board is already using `TIMER2`.
- Confirming the `--wrap=bt_le_adv_start*` symbol names still exist in the
  host version you're linking against.

## Worth knowing

- Takes over `RADIO_IRQn` while active. `bt_le_adv_start` and friends are
  linker-wrapped so ZMK can keep thinking advertising is running — the
  normal BLE path resumes cleanly when the user switches away from the
  ESB slot.
- While active, the HID listener returns `ZMK_EV_EVENT_HANDLED`, so USB
  and BLE HID stay silent for keycode/consumer events. ESB is the only
  output.
- Only tested against ZMK v0.3.0.

## License

MIT for module sources. Vendored NCS ESB files are LicenseRef-Nordic-5-Clause —
see `vendor/nrf-esb/LICENSE`.
