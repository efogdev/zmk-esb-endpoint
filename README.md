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

## Dongle ([example firmware](https://github.com/efogtech/endgame-trackball-firmware/tree/main/dongle-1k-firmware))

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

> [!IMPORTANT]
> The shell relay needs the [`zmk-shell-relay-core`](https://github.com/efogdev/zmk-shell-relay-core)
> module present in your `west.yml`. It owns the shared internal shell
> instance and output buffer; without it the relay will not build.

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
Persistent quarantine (`CHANNEL_QUARANTINE_PERSIST`) counts how often
each channel triggers a fail-driven hop and long-term avoids repeat
offenders, persisting that set across reboots with configurable decay.
Channel selection is also weighted by per-channel RSSI EWMAs
(`CHANNEL_RSSI_WEIGHT`) so channels with historically better signal are
preferred. Idle links never hop on their own (the periodic `IDLE` packet
tells the dongle to disarm its silence watchdog). A post-hop quiet window
(`POST_QUIET_MS`) suppresses TX so the dongle's speculative hop can
catch up; a short PROPOSAL burst on the new channel — including
periodic re-anchors on the DTS-default rendezvous channel — re-syncs
a dongle that missed the hop entirely. If TX never recovers within
`RENDEZVOUS_FALLBACK_MS` the endpoint forces a hop back to the
rendezvous channel, bypassing both quarantine and cooldown, then holds
there for `RENDEZVOUS_HOLD_MS` so the dongle's rollback dwell has a
stationary target to re-acquire.

The `esb stats` shell command prints the current channel, committed
next, hop active/idle state, RSSI, per-channel RSSI table, link-quality
window, and per-link TX / HID counters.

## Link benchmark (optional)

`CONFIG_ZMK_ESB_ENDPOINT_BENCH=y` adds an `esb bench` shell command.
First invocation blasts `BENCH_PING` for `BENCH_DURATION_S` seconds;
second invocation prints throughput (pkt/s, ok/fail) and RSSI (avg/min/max)
reported back from the dongle via ACK payload. 

## Link health and recovery

HFXO is held for the whole ESB slot so the LFRC calibrator can never stop the crystal between packets.
While ESB owns the radio, the BT stack's PPI channels are cleared — the LL drives RADIO tasks via TIMER0
shortcuts, and leaving those live lets a stray CC event clobber ESB's channel or address config mid-packet.
The radio runs in fast-ramp-up mode (RXIDLE→RX in ~40 µs instead of ~140 µs), shrinking the per-retry window.
TX power is set to the hardware maximum (8 dBm). The retransmit delay is jittered ±12.5% per send so retries
don't phase-lock with Wi-Fi beacon intervals. LFRC calibration is forced on every channel hop — stale LF timing
is a plausible contributor after a fail streak, and the call is async so it doesn't block the hop.

`retransmit_count` is adjusted continuously from an EWMA of tx_attempts on the success path (TX_FAILED samples
are excluded — they're capped by the hardware ceiling and would contaminate the average). On a clean link it
stays at `COUNT_MIN` for low tail latency; as the EWMA climbs it ramps toward `COUNT_MAX`. Control packets where
a loss is costly — BEACON, DISCONNECT, VERIFY_REQ, HOP_OFFER, IDLE, CHANNEL_HOP_PROPOSAL — bypass the adaptive
value entirely and transmit at `CRITICAL_RETRANSMIT_COUNT` (default 14).

Motion-only mouse reports use a lower `POINTER_RETRANSMIT_COUNT`. On TX_FAILED their deltas go into a refund
pool (saturating int16) and are merged into the next outgoing mouse report rather than being lost. Reports that
also carry button bits keep the global count — buttons are edge-triggered and can't be reconstructed by
accumulation. A per-instance in-flight cap (`POINTER_INFLIGHT_CAP`) back-pressures the input processor before
the ESB FIFO fills with frames that are already stale.

Before dropping the TX FIFO (on idle transition or channel hop), the transport waits up to `FLUSH_QUIET_MS` for
the radio to go silent so any in-flight packet can finish its ACK cycle instead of being wasted. `FLUSH_FORCE_MS`
caps the total wait so the path doesn't stall indefinitely under sustained traffic.

Keyboard and consumer HID edges that arrive during a transport-quiet window (post-hop blackout, rendezvous
side-trip) are held in snapshot rings and replayed in arrival order once the window clears — the same idea as
mouse button edge queuing described in the Dongle section, but for keycode and consumer events where a missed
release would strand a modifier or media key on the host.

The HID relay tracks explicit and implicit modifiers in separate per-bit refcounts, decoupled from ZMK's global
HID state. This blocks ZMK's known implicit-mod cross-release bug from affecting the ESB stream — a modifier
that ZMK would incorrectly drop stays held in the ESB-local state until its own key-up event arrives.

Link quality is measured via a sliding-window popcount of recent ACKed TX events (each bit set if the event
needed a retransmit) and an EWMA of tx_attempts on the success path. Both metrics exclude noack sends via a
parallel ring — noack events report tx_attempts=1 unconditionally and would pull the numbers toward zero.

If a hop produces no TX_SUCCESS before the fail threshold fires again, the endpoint reverts to the channel it just
left rather than picking a third. A second immediate failure strongly suggests the original channel was better, or at least no worse.

## Configuration

Core:

| Kconfig                                                           | What it does                                                                                                                                                     |
|-------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `ZMK_ESB_ENDPOINT_LOG_LEVEL`                                      | Compile-time log level for all module TUs (0=OFF…4=DBG, default 2=WRN). Independent of `CONFIG_ZMK_LOG_LEVEL`.                                                   |
| `ZMK_ESB_ENDPOINT_SHELL_CMDS`                                     | Register the `esb` shell command set (unpair, channel, bench). On by default; disable to save ROM.                                                               |
| `ZMK_ESB_ENDPOINT_HID_NOACK`                                      | Mouse fire-and-forget (default off).                                                                                                                             |
| `ZMK_ESB_ENDPOINT_RETRANSMIT_COUNT` / `_DELAY_US`                 | ACKed-packet retransmit policy (default 5 retries, 610 µs delay).                                                                                                |
| `ZMK_ESB_ENDPOINT_POINTER_RETRANSMIT_COUNT`                       | Lower retransmit count for motion-only mouse reports (default 4). Motion loss is recovered by delta accumulation; button reports always use the global count.    |
| `ZMK_ESB_ENDPOINT_POINTER_INFLIGHT_CAP`                           | Max in-flight pointer-motion reports queued to ESB before backpressure kicks in (default 5).                                                                     |
| `ZMK_ESB_ENDPOINT_CRITICAL_MAX_RETRANSMIT`                        | Use an elevated retransmit count (`CRITICAL_RETRANSMIT_COUNT`, default 14) for control packets (BEACON, DISCONNECT, VERIFY_REQ, HOP_OFFER, IDLE). On by default. |
| `ZMK_ESB_ENDPOINT_BEACON_INTERVAL_MS`                             | Beacon rate while unpaired.                                                                                                                                      |
| `ZMK_ESB_ENDPOINT_BEACON_INITIAL_DELAY_MS`                        | Delay before first beacon after activate / unpair / disconnect.                                                                                                  |
| `ZMK_ESB_ENDPOINT_VERIFY_INTERVAL_MS`                             | Identity `VERIFY_REQ` retransmit cadence during reconnect.                                                                                                       |
| `ZMK_ESB_ENDPOINT_BLE_QUIESCE_MS`                                 | Quiet time after adv-stop before radio swap.                                                                                                                     |
| `ZMK_ESB_ENDPOINT_BOOT_CHECK_DELAY_MS`                            | Settings-load delay before polling the boot profile.                                                                                                             |
| `ZMK_ESB_ENDPOINT_CTRL_THREAD_PRIORITY`                           | Cooperative priority of the ESB control thread.                                                                                                                  |
| `ZMK_ESB_ENDPOINT_CTRL_STACK_SIZE`                                | ESB control thread stack size in bytes (default 1280).                                                                                                           |
| `ZMK_ESB_ENDPOINT_CTRL_MSGQ_DEPTH`                                | Activate/deactivate command queue depth.                                                                                                                         |
| `ZMK_ESB_ENDPOINT_TX_FAIL_WARN_THRESHOLD`                         | Consecutive TX fails before WRN log.                                                                                                                             |
| `ZMK_ESB_ENDPOINT_TX_FAIL_ERR_THRESHOLD`                          | Consecutive TX fails before ERR log + TX flush.                                                                                                                  |
| `ZMK_ESB_ENDPOINT_HID_QUIET_KB_QUEUE_DEPTH` / `_CONS_QUEUE_DEPTH` | Per-edge keyboard / consumer HID snapshot ring depth for transport-quiet windows (default 8 / 5).                                                                |
| `ZMK_ESB_ENDPOINT_HID_QUIET_RETRY_MS`                             | HID drain poll cadence while a quiet window is open (default 1 ms).                                                                                              |
| `ZMK_ESB_ENDPOINT_IP_PENDING_BTN_QUEUE`                           | Input-processor pending button-edge ring depth during quiet / backpressure (default 8).                                                                          |
| `ZMK_ESB_ENDPOINT_IP_QUIET_RETRY_MS`                              | Input-processor quiet-drain poll cadence (default 1 ms).                                                                                                         |
| `ZMK_ESB_ENDPOINT_FLUSH_QUIET_MS` / `_FORCE_MS`                   | Pre-flush radio quiet wait and its upper-bound ceiling (default 2 ms / 3 ms).                                                                                    |

Adaptive retry (`ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY`, default on, requires `CHANNEL_HOP`):

| Kconfig                                       | What it does                                                                                                                                      |
|-----------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------|
| `ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY`             | Periodically retune `retransmit_count` from a link-quality EWMA: lower on a clean link, higher on a degraded one. Saves ~150–200 B when disabled. |
| `..._ADAPTIVE_RETRY_INTERVAL_MS`              | Re-evaluation cadence (default 25 ms).                                                                                                            |
| `..._ADAPTIVE_RETRY_EWMA_LOW` / `_HIGH`       | EWMA breakpoints (in tx_attempts × 10) below/above which the count is pinned at `COUNT_MIN` / `COUNT_MAX` (default 12 / 30).                      |
| `..._ADAPTIVE_RETRY_COUNT_MIN` / `_COUNT_MAX` | Floor / ceiling for the adaptive retransmit_count (default 3 / 12).                                                                               |

Shell relay (`ZMK_ESB_ENDPOINT_SHELL_RELAY`):

| Kconfig                                   | What it does                                                                                   |
|-------------------------------------------|------------------------------------------------------------------------------------------------|
| `..._SHELL_POLL_INTERVAL_MS`              | Poll rate while session active.                                                                |
| `..._SHELL_BG_POLL`                       | Master toggle for the idle keepalive (default on).                                             |
| `..._SHELL_BG_POLL_MS`                    | Poll rate while paired but idle.                                                               |
| `..._SHELL_BG_POLL_ACTIVITY_THRESHOLD_MS` | Suppress idle poll if no ESB TX/RX in this long (0 = always poll).                             |
| `..._SHELL_INITIAL_POLL_DELAY_MS`         | Delay before the first active poll after `SHELL_REQ`.                                          |
| `..._SHELL_INACTIVITY_S`                  | Idle timeout before auto `SHELL_STOP`.                                                         |
| `..._SHELL_CMD_BUF_SIZE`                  | Command assembly buffer.                                                                       |
| `..._SHELL_STDOUT_COALESCE_BYTES`         | stdout hook coalescing buffer: flush when this many bytes accumulate (default 96).             |
| `..._SHELL_STDOUT_COALESCE_MS`            | Flush the stdout coalescing buffer after this idle period (default 5 ms).                      |
| `..._SHELL_LOG_LINE_BUF_SIZE`             | Per-message log staging buffer size (default 96 B).                                            |
| `..._SHELL_STOP_NUDGE_COOLDOWN_MS`        | Minimum interval between `SHELL_STOP` nudges when the dongle is out of sync (default 5000 ms). |
| `..._SHELL_PROMPT`                        | Prompt string sent to the dongle.                                                              |

Channel hopping (`ZMK_ESB_ENDPOINT_CHANNEL_HOP`):

| Kconfig                                                      | What it does                                                                                                                      |
|--------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------|
| `ZMK_ESB_ENDPOINT_COOP_HOP`                                  | Enable the `HOP_OFFER` / `HOP_ACCEPT` cooperative-hop handshake (default on). Saves ~1.0–1.2 KB when disabled.                    |
| `ZMK_ESB_ENDPOINT_RENDEZVOUS`                                | Enable the rendezvous side-trip and cooldown-fallback recovery paths (default on). Saves ~550 B when disabled.                    |
| `..._CHANNEL_HOP_TX_FAIL_WINDOW_MS`                          | Continuous-fail duration that fires a dead-link hop.                                                                              |
| `..._CHANNEL_HOP_WEAK_LINK_MS`                               | No-`TX_SUCCESS` duration that fires a weak-link hop (0 = off).                                                                    |
| `..._CHANNEL_HOP_POST_QUIET_MS`                              | Quiet window after a hop before TX resumes.                                                                                       |
| `..._MOTION_MAX_STALE_MS`                                    | Max age of accumulated pointer deltas before they are dropped instead of flushed.                                                 |
| `..._CHANNEL_HOP_COOLDOWN_MS`                                | Minimum dwell before another TX-fail hop can fire.                                                                                |
| `..._CHANNEL_QUARANTINE_MS`                                  | How long a recently-bad channel stays excluded.                                                                                   |
| `..._CHANNEL_QUARANTINE_MIN_DISTANCE`                        | MHz of guard around any quarantined channel.                                                                                      |
| `..._CHANNEL_QUARANTINE_PERSIST`                             | Count per-channel fail-driven hops and persistently avoid the worst offenders across reboots.                                     |
| `..._CHANNEL_QUARANTINE_PERSIST_SIZE`                        | Number of worst-offender channels to remember (default 5).                                                                        |
| `..._CHANNEL_QUARANTINE_PERSIST_THRESHOLD`                   | Hits required before a channel enters the persisted avoid set (default 3).                                                        |
| `..._CHANNEL_QUARANTINE_PERSIST_INTERVAL_MS`                 | Distil + NVS write cadence (default 1 200 000 ms).                                                                                |
| `..._CHANNEL_QUARANTINE_PERSIST_DECAY_S`                     | Decay one hit per this many powered-on seconds (default 86 400; 0 = no decay).                                                    |
| `..._CHANNEL_RSSI_WEIGHT`                                    | Weight channel selection by per-channel RSSI EWMAs (default on).                                                                  |
| `..._CHANNEL_RSSI_WEIGHT_SCALE` / `_MIN` / `_MAX`            | Fixed-point scale and weight range for RSSI-weighted picking (default 1000 / 500 / 2000).                                         |
| `..._CHANNEL_RSSI_EWMA_ALPHA`                                | EWMA smoothing factor for per-channel RSSI (default 200 × SCALE⁻¹).                                                               |
| `..._CHANNEL_RSSI_PERSIST_INTERVAL_MS`                       | Distil + NVS write cadence for RSSI EWMAs (default 1 200 000 ms).                                                                 |
| `..._CHANNEL_HOP_NEGOTIATE_INTERVAL_MS`                      | Steady-state PROPOSAL cadence (battery-friendly).                                                                                 |
| `..._CHANNEL_HOP_NEGOTIATE_RETRY_MS`                         | Faster cadence while `committed_next` is missing.                                                                                 |
| `..._CHANNEL_HOP_REQUEST_BURST_COUNT`                        | Fast-retry attempts seeded by an inbound `REQUEST`.                                                                               |
| `..._CHANNEL_HOP_POST_BURST_COUNT` / `_INTERVAL_MS`          | Post-hop PROPOSAL burst sizing (0 = off).                                                                                         |
| `..._CHANNEL_HOP_POST_BURST_RENDEZVOUS_EVERY`                | Every Nth burst tick goes to the rendezvous channel.                                                                              |
| `..._CHANNEL_HOP_RENDEZVOUS_TIMEOUT_MS`                      | Sync-TX timeout for the rendezvous side-trip.                                                                                     |
| `..._CHANNEL_HOP_RENDEZVOUS_FALLBACK_MS`                     | Force-hop to rendezvous after this long with no recovery (0 = off).                                                               |
| `..._CHANNEL_HOP_RENDEZVOUS_HOLD_MS`                         | Hold on the rendezvous channel after a fallback before allowing hops away (default 685 ms; 0 = disable).                          |
| `..._IDLE_THRESHOLD_MS`                                      | No-user-TX duration before declaring the endpoint idle.                                                                           |
| `..._LINK_QUALITY_WINDOW`                                    | Sliding-window size for the cooperative-hop trigger.                                                                              |
| `..._LINK_QUALITY_NOACK_RING_SIZE`                           | Per-send ring depth for noack-flag tracking so noack events don't dilute link-quality EWMAs (default 16; must be a power of two). |
| `..._LINK_DEGRADED_THRESHOLD` / `_REARM` / `_FAST_THRESHOLD` | Cooperative-hop fire / hysteresis / fast-cliff thresholds.                                                                        |
| `..._COOP_HOP_HOP_IN_MS`                                     | Synchronised commit deadline for `HOP_OFFER`.                                                                                     |
| `..._COOP_HOP_COOLDOWN_MS`                                   | Minimum interval between two cooperative hops.                                                                                    |
| `..._COOP_HOP_QUARANTINE_MS`                                 | Shorter quarantine for coop hops (often transient).                                                                               |
| `..._HOP_RETRY_DELAY_MS`                                     | Re-arm delay after `esb_transport_set_channel` returns `-EBUSY` (default 2 ms with `CRITICAL_MAX_RETRANSMIT`, 1 ms otherwise).    |
| `..._HOP_STOP_TX_AFTER_BUSY`                                 | Consecutive `-EBUSY` attempts before forcing the radio idle (default 2).                                                          |

Benchmark (`ZMK_ESB_ENDPOINT_BENCH`):

| Kconfig                       | What it does                          |
|-------------------------------|---------------------------------------|
| `..._BENCH_DURATION_S`        | How long to blast pings.              |
| `..._BENCH_RESULT_POLL_MS`    | Poll interval while draining results. |
| `..._BENCH_RESULT_TIMEOUT_MS` | Max wait for dongle result.           |

ESB transport defaults tuned by this module (in `src/Kconfig`): max
payload length 32, pipe count 2, `ESB_NEVER_DISABLE_TX=y`; TX FIFO depth
and RX FIFO depth both default to 8 and are bumped to 32 / 24
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
