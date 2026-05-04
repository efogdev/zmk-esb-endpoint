# zmk-esb-endpoint

Turns the last BLE profile slot on a ZMK keyboard into an ESB PTX endpoint
that talks to a matching USB dongle. Pick that profile and ESB takes the
radio; pick a different one and BLE comes back.

> [!CAUTION]
> This module does not support ZMK split topology (yet?). Works only with unibody devices.

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

Add to devicetree:
```dts
/ {
	zmk_esb: zmk_esb_endpoint {
		compatible = "zmk,esb-endpoint";
		esb-channel = <78>;
		pairing-base-address = [17 f4 07 aa];
		pairing-prefix = <0x24>;
		data-base-address = [b9 8a 16 22];
		data-prefix = <0xc2>;
	};

	esb_ip: esb_input_processor {
		compatible = "zmk,esb-input-processor";
		#input-processor-cells = <0>;
		status = "okay";
	};
};
```

Relay the pointer events to the endpoint:
```dts
&mkp_input_listener { input-processors = <&esb_ip>; };
&msc_input_listener { input-processors = <&esb_ip>; };
&your_pointer_listener { input-processors = <&esb_ip>; };
```

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
Core HID path is five message types (`BEACON` / `PAIR_REQ` / `PAIR_RESP` /
`HID_REPORT` / `DISCONNECT`); HID report bodies are ZMK's `zmk_hid_*_report_body`
structs copied verbatim into an ESB payload, so the dongle can hand them
straight to USB HID with nothing more than the report-id prefix byte. The
PAIR_REQ ACK is expected to carry the dongle's 6-byte `device_id`, and a
paired dongle must answer a `VERIFY_REQ` (pipe 1) with a `VERIFY_RESP` ACK
containing the same id — this is how the keyboard confirms on reconnect
that it's still talking to the dongle it paired with. An unpaired dongle
that receives `VERIFY_REQ` should answer with `DISCONNECT` so the keyboard
unpairs and re-beacons cleanly. A `RESYNC` ACK from the dongle drops the
keyboard from CONNECTED back to VERIFYING without wiping the stored peer,
so a dongle that rebooted mid-session can re-handshake without forcing a
full re-pair.

Beyond pairing, the keyboard also emits `IDLE` after `IDLE_THRESHOLD_MS`
of no user TX (so the dongle can disarm its silence watchdog and stop
hunting for the keyboard), and consumes `LINK_STATS` ACKs carrying the
dongle's RSSI snapshot for adaptive decisions. With channel hopping
enabled, both sides also exchange `CHANNEL_HOP_PROPOSAL` / `_CONFIRM` /
`_REQUEST` and the cooperative `HOP_OFFER` / `HOP_ACCEPT` pair — see
the Channel hopping section below.

Mouse reports are sent with `noack=1` when `ZMK_ESB_ENDPOINT_HID_NOACK=y` —
a dropped pointer frame is self correcting. Keyboard and consumer reports
are always ACKed; a lost release packet would strand a key on the host.
Mouse motion accumulated during a transport quiet window (post-hop / sync
side-trip) is dropped if it ages past
`ZMK_ESB_ENDPOINT_MOTION_MAX_STALE_MS` so a flush doesn't teleport the
cursor by tens of stale deltas. Button edges that fall in the same
window are queued (small fixed depth, oldest-first eviction on overrun)
and replayed in order once the window ends, so press/release pairs land
on the host even when the radio was unavailable.

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
the keyboard sends a periodic `SHELL_BG_POLL` (toggle with
`SHELL_BG_POLL=n`) so a pending dongle request arrives promptly via ACK
payload; `SHELL_BG_POLL_ACTIVITY_THRESHOLD_MS` gates that poll on recent
ESB traffic so a fully idle link goes silent.

## Channel hopping (optional)

`CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP=y` (default on) lets the link move
off a noisy 2.4 GHz channel without re-pairing. While the link is healthy
the endpoint negotiates a "committed next channel" with the dongle via
`CHANNEL_HOP_PROPOSAL` / `CHANNEL_HOP_CONFIRM` (and `_REQUEST` for the
dongle to nudge the endpoint when its own committed_next is empty). A
hop fires on three independent triggers:

- **Dead-link**: TX has been failing continuously for
  `CHANNEL_HOP_TX_FAIL_WINDOW_MS`.
- **Weak-link**: no `TX_SUCCESS` for `CHANNEL_HOP_WEAK_LINK_MS` even
  though some packets squeak through.
- **Cooperative**: a sliding window of recent ACKed packets shows
  `LINK_DEGRADED_THRESHOLD` retransmits — the endpoint sends `HOP_OFFER`
  with a synchronised commit deadline; the dongle replies with
  `HOP_ACCEPT` in the ACK and both sides flip the radio within ~500 µs
  of each other, minimising blackout.

The channel just left is quarantined for `CHANNEL_QUARANTINE_MS` and
new candidates avoid quarantined channels by `..._MIN_DISTANCE` MHz.
Idle links never hop on their own (the periodic `IDLE` packet tells the
dongle to disarm its silence watchdog). A post-hop quiet window
(`POST_QUIET_MS`) suppresses TX so the dongle's speculative hop can
catch up; a short PROPOSAL burst on the new channel — including
periodic re-anchors on the DTS-default rendezvous channel — re-syncs
a dongle that missed the hop entirely. If TX never recovers within
`RENDEZVOUS_FALLBACK_MS` the endpoint forces a hop back to the
rendezvous channel, bypassing both quarantine and cooldown.

The `esb stats` shell command prints the current channel, committed
next, hop active/idle state, RSSI, link-quality window, and per-link
TX / HID counters.

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
| `ZMK_ESB_ENDPOINT_RETRANSMIT_COUNT` / `_DELAY_US` | ACKed-packet retransmit policy (default 5 retries, 570 µs delay). |
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
| `..._SHELL_BG_POLL`                 | Master toggle for the idle keepalive (default on). |
| `..._SHELL_BG_POLL_MS`              | Poll rate while paired but idle. |
| `..._SHELL_BG_POLL_ACTIVITY_THRESHOLD_MS` | Suppress idle poll if no ESB TX/RX in this long (0 = always poll). |
| `..._SHELL_INITIAL_POLL_DELAY_MS`   | Delay before the first active poll after `SHELL_REQ`. |
| `..._SHELL_INACTIVITY_S`            | Idle timeout before auto `SHELL_STOP`. |
| `..._SHELL_CMD_BUF_SIZE`            | Command assembly buffer. |
| `..._SHELL_OUT_BUF_SIZE`            | TX ring buffer for shell output. |
| `..._SHELL_PROMPT`                  | Prompt string sent to the dongle. |

Channel hopping (`ZMK_ESB_ENDPOINT_CHANNEL_HOP`):

| Kconfig | What it does |
|---------|--------------|
| `..._CHANNEL_HOP_TX_FAIL_WINDOW_MS` | Continuous-fail duration that fires a dead-link hop. |
| `..._CHANNEL_HOP_WEAK_LINK_MS`      | No-`TX_SUCCESS` duration that fires a weak-link hop (0 = off). |
| `..._CHANNEL_HOP_POST_QUIET_MS`     | Quiet window after a hop before TX resumes. |
| `..._MOTION_MAX_STALE_MS`           | Max age of accumulated pointer deltas before they are dropped instead of flushed. |
| `..._CHANNEL_HOP_COOLDOWN_MS`       | Minimum dwell before another TX-fail hop can fire. |
| `..._CHANNEL_QUARANTINE_MS`         | How long a recently-bad channel stays excluded. |
| `..._CHANNEL_QUARANTINE_MIN_DISTANCE` | MHz of guard around any quarantined channel. |
| `..._CHANNEL_HOP_NEGOTIATE_INTERVAL_MS` | Steady-state PROPOSAL cadence (battery-friendly). |
| `..._CHANNEL_HOP_NEGOTIATE_RETRY_MS` | Faster cadence while `committed_next` is missing. |
| `..._CHANNEL_HOP_REQUEST_BURST_COUNT` | Fast-retry attempts seeded by an inbound `REQUEST`. |
| `..._CHANNEL_HOP_POST_BURST_COUNT` / `_INTERVAL_MS` | Post-hop PROPOSAL burst sizing (0 = off). |
| `..._CHANNEL_HOP_POST_BURST_RENDEZVOUS_EVERY` | Every Nth burst tick goes to the rendezvous channel. |
| `..._CHANNEL_HOP_RENDEZVOUS_TIMEOUT_MS` | Sync-TX timeout for the rendezvous side-trip. |
| `..._CHANNEL_HOP_RENDEZVOUS_FALLBACK_MS` | Force-hop to rendezvous after this long with no recovery (0 = off). |
| `..._IDLE_THRESHOLD_MS`             | No-user-TX duration before declaring the endpoint idle. |
| `..._LINK_QUALITY_WINDOW`           | Sliding-window size for the cooperative-hop trigger. |
| `..._LINK_DEGRADED_THRESHOLD` / `_REARM` / `_FAST_THRESHOLD` | Cooperative-hop fire / hysteresis / fast-cliff thresholds. |
| `..._COOP_HOP_HOP_IN_MS`            | Synchronised commit deadline for `HOP_OFFER`. |
| `..._COOP_HOP_COOLDOWN_MS`          | Minimum interval between two cooperative hops. |
| `..._COOP_HOP_QUARANTINE_MS`        | Shorter quarantine for coop hops (often transient). |

Benchmark (`ZMK_ESB_ENDPOINT_BENCH`):

| Kconfig | What it does |
|---------|--------------|
| `..._BENCH_DURATION_S`              | How long to blast pings. |
| `..._BENCH_RESULT_POLL_MS`          | Poll interval while draining results. |
| `..._BENCH_RESULT_TIMEOUT_MS`       | Max wait for dongle result. |

ESB transport defaults tuned by this module (in `src/Kconfig`): max
payload length 32, pipe count 2, `ESB_NEVER_DISABLE_TX=y`; TX FIFO depth
and RX FIFO depth both default to 8 and are bumped to 64 / 32
respectively when the shell relay is enabled.

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

**nRF52840 should work but I'm not sure.** Review the PPI mask against your LL build.

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
