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
PAIR beacons; press the pair button on the dongle to bind. The dongle
advertises its FICR device_id in the PAIR_REQ ACK; the keyboard persists
it alongside the paired flag. Subsequent boots enter a short VERIFYING
state — HID stays suppressed until the dongle's live device_id matches the
stored one via a `VERIFY_REQ` / `VERIFY_RESP` exchange. A stranger dongle
(or a stale mismatched record) logs a warning and never flips to
CONNECTED, so input can't leak to the wrong host.

## Dongle

You bring your own PRX. The wire protocol is in `include/zmk_esb/protocol.h`.
Core HID path is five message types (BEACON / PAIR_REQ / PAIR_RESP /
HID_REPORT / DISCONNECT); HID report bodies are ZMK's `zmk_hid_*_report_body`
structs copied verbatim into an ESB payload, so the dongle can hand them
straight to USB HID with nothing more than the report-id prefix byte. The
PAIR_REQ ACK is expected to carry the dongle's 6-byte `device_id`, and a
paired dongle must answer a `VERIFY_REQ` (pipe 1) with a `VERIFY_RESP` ACK
containing the same id — this is how the keyboard confirms on reconnect
that it's still talking to the dongle it paired with. An unpaired dongle
that receives `VERIFY_REQ` should answer with `DISCONNECT` so the keyboard
unpairs and re-beacons cleanly.

Mouse reports are sent with `noack=1` when `ZMK_ESB_ENDPOINT_HID_NOACK=y` —
a dropped pointer frame is self correcting. Keyboard and consumer reports
are always ACKed; a lost release packet would strand a key on the host.

## Keymap behaviors

`&esb_unpair` forgets the paired dongle and restarts beaconing. Include
`dts/behaviors/esb_unpair.dtsi` from your keymap and bind it like any
other behavior. Enabled by default with the module
(`CONFIG_ZMK_ESB_BEHAVIOR_UNPAIR=y`). The same action is available as
the `esb unpair` shell command.

`&esb_shell_req` asks the dongle to open a shell relay session — the
keyboard analogue of the dongle's short pair-button press. Include
`dts/behaviors/esb_shell_req.dtsi` and bind the behavior to any key.
No-ops when the ESB endpoint is not the active output or when a
session is already open. Enabled by default with the shell relay
(`CONFIG_ZMK_ESB_BEHAVIOR_SHELL_REQ=y`, depends on
`ZMK_ESB_ENDPOINT_SHELL_RELAY`).

## Shell relay (optional)

`CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY=y` turns the link into a bidirectional
Zephyr shell transport: the dongle requests a session (`SHELL_REQ` via ACK
payload), the keyboard executes commands through the dummy shell backend
and streams output back as `SHELL_DATA`. A small ring buffer absorbs
backpressure when the ESB TX FIFO is full and drains on `TX_SUCCESS`.
Sessions end on `SHELL_INACTIVITY_S` of no input. While paired but idle,
the keyboard sends a periodic `SHELL_BG_POLL` so a pending dongle request
arrives promptly via ACK payload. 

## Link benchmark (optional)

`CONFIG_ZMK_ESB_ENDPOINT_BENCH=y` adds an `esb bench` shell command.
First invocation blasts `BENCH_PING` for `BENCH_DURATION_S` seconds;
second invocation prints throughput (pkt/s, ok/fail) and RSSI (avg/min/max)
reported back from the dongle via ACK payload. 

## Configuration

Core:

| Kconfig                                        | What it does |
|------------------------------------------------|--------------|
| `ZMK_ESB_ENDPOINT_HID_NOACK`                   | Mouse fire-and-forget (default off). |
| `ZMK_ESB_ENDPOINT_RETRANSMIT_COUNT` / `_DELAY_US` | ACKed-packet retransmit policy (default delay 350us). |
| `ZMK_ESB_ENDPOINT_BEACON_INTERVAL_MS`          | Beacon rate while unpaired. |
| `ZMK_ESB_ENDPOINT_BEACON_INITIAL_DELAY_MS`     | Delay before first beacon after activate / unpair / disconnect. |
| `ZMK_ESB_ENDPOINT_VERIFY_INTERVAL_MS`          | Identity `VERIFY_REQ` retransmit cadence during reconnect. |
| `ZMK_ESB_ENDPOINT_BLE_QUIESCE_MS`              | Quiet time after adv-stop before radio swap. |
| `ZMK_ESB_ENDPOINT_BOOT_CHECK_DELAY_MS`         | Settings-load delay before polling the boot profile. |
| `ZMK_ESB_ENDPOINT_CTRL_THREAD_PRIORITY`        | Cooperative priority of the ESB control thread. |
| `ZMK_ESB_ENDPOINT_CTRL_MSGQ_DEPTH`             | Activate/deactivate command queue depth. |
| `ZMK_ESB_ENDPOINT_TX_FAIL_WARN_THRESHOLD`      | Consecutive TX fails before WRN log. |
| `ZMK_ESB_ENDPOINT_TX_FAIL_ERR_THRESHOLD`       | Consecutive TX fails before ERR log + TX flush. |

Shell relay (`ZMK_ESB_ENDPOINT_SHELL_RELAY`):

| Kconfig | What it does |
|---------|--------------|
| `..._SHELL_POLL_INTERVAL_MS`        | Poll rate while session active. |
| `..._SHELL_BG_POLL_MS`              | Poll rate while paired but idle. |
| `..._SHELL_INACTIVITY_S`            | Idle timeout before auto `SHELL_STOP`. |
| `..._SHELL_CMD_BUF_SIZE`            | Command assembly buffer. |
| `..._SHELL_OUT_BUF_SIZE`            | TX ring buffer for shell output. |
| `..._SHELL_PROMPT`                  | Prompt string sent to the dongle. |

Benchmark (`ZMK_ESB_ENDPOINT_BENCH`):

| Kconfig | What it does |
|---------|--------------|
| `..._BENCH_DURATION_S`              | How long to blast pings. |
| `..._BENCH_RESULT_POLL_MS`          | Poll interval while draining results. |
| `..._BENCH_RESULT_TIMEOUT_MS`       | Max wait for dongle result. |

ESB transport defaults tuned by this module (in `src/Kconfig`): payload
length 24, pipe count 2, `ESB_NEVER_DISABLE_TX=y`; TX/RX FIFO depth is
bumped to 24 when the shell relay is enabled.

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
