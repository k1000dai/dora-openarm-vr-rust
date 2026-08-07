# dora-openarm-vr-rust

Behavior-compatible Rust port of [`enactic/dora-openarm-vr`](https://github.com/enactic/dora-openarm-vr).

The repository and crate use the `-rust` suffix. The installed binary deliberately retains the upstream console-script name, `dora-openarm-quest-receiver`, so existing dora dataflows can replace the Python node without changing `path:`. Upstream ships that binary under `LICENSE.txt`; this port uses the conventional `LICENSE` filename with the same Apache-2.0 text.

## Compatibility

Ports `dora_openarm_vr.quest_receiver`, `.smoothing`, and `.udp_receiver` from upstream. On every `tick` input:

- Drains the UDP background thread's queued packet-arrival timestamps and, if any arrived, publishes `vr_receive_times` (`Int64Array`, no `timestamp` metadata, matching upstream sending it with no `ts` argument).
- If no packet has ever been received, does nothing further this tick.
- Converts the latest packet's Unity left-handed `rc`/`lc`/`rf` poses to the robot's right-handed `arm_origin` frame: flip `z`/`qx`/`qy`, normalize, rectify relative to the reference pose (identity when `rf` is absent), rotate into the frame via a fixed `[[0,0,-1],[-1,0,0],[0,1,0]]` matrix (hardcoded as the quaternion `[0.5,-0.5,-0.5,0.5]`, verified against scipy 1.17.1/numpy 2.4.6 -- the versions upstream's `pyproject.toml` requires), add the `[-0.085, 0, -0.14]` neutral offset, and apply a 90 degree Z-axis fix rotation. All of this runs in `f64`; only the final packed pose narrows to `f32`.
- Smooths `pose_right`/`pose_left`/`pose_reference` independently with a One Euro filter (`min_cutoff=2.0, beta=0.04, d_cutoff=1.5`), gated by each pose's own Quest validity code (`v`/`vr`/`vl`; default `OK` when absent). An `INVALID` code suppresses that pose's output and resets its smoother exactly once, on the `valid -> INVALID` transition, so the first sample after recovery passes through unfiltered.
- Publishes, in upstream's exact order, whichever of these are gated in for this tick:
  1. `vr_receive_times` -- `Int64Array`, only when non-empty.
  2. `pose_right` -- `Struct{pose: List<Float32>}` len 8 (`[x,y,z,qw,qx,qy,qz,gripper]`), only when the right pose is valid *and* the packet carries `rt`.
  3. `pose_left` -- same shape, gated on the left pose and `lt`.
  4. `pose_reference` -- `Struct{pose: List<Float32>}` len 7 (no gripper), gated only on overall validity and `rf` being present (no trigger required).
  5. `trigger_right`, `trigger_left`, `grip_right`, `grip_left` -- `Float32Array` len 1, each gated only on its own field's presence (independent of pose validity).
  6. `joystick_x_left`, `joystick_y_left`, `joystick_x_right`, `joystick_y_right` -- `Float32Array` len 1.
  7. `button_a`, `button_b`, `button_x`, `button_y` -- `BooleanArray` len 1, Python-truthiness of the field's JSON value.
- The gripper angle appended to `pose_right`/`pose_left` maps the trigger (clipped to `[0, 1]`) to a calibrated angle: right `-45 deg -> 10 deg`, left `45 deg -> -10 deg`.
- Every gated output above carries a `timestamp` (ns) metadata key; `vr_receive_times` and the one-time startup `status` do not.
- Publishes `status = "ready"` once, immediately after connecting to dora, before the first tick.
- Logs `[receiver] validity: OLD -> NEW (L=..., R=...)` to stdout whenever the overall validity code changes, matching upstream's diagnostic print (not a dora output).
- Reacts only to `tick` inputs; all other event types (including `Stop`) are ignored the same way upstream's `for event in node: if event["type"] != "INPUT" or event["id"] != "tick": continue` ignores them -- there is no dedicated `STOP` handling on either side.
- `--host` (default `0.0.0.0`) / `--port` (default `5006`): upstream's `quest_receiver.py` reads these only from `argparse`, with no environment variable fallback, so this port doesn't add one either.
- `--max-linear-speed` (default `1.0` m/s) and `--max-angular-speed` (default `6.0` rad/s): cap per-tick controller translation/rotation steps after One Euro filtering; pass `0` to disable either limit. Invalid (negative, NaN, or infinity) values are rejected.
- The UDP background thread is a close port of `JsonUdpReceiver`: it binds (retrying every second on failure), blocks on `recv_from` with a 1 second timeout, and drains any additional already-queued datagrams non-blockingly before waiting again. Only successfully-parsed datagrams update the latest message or the (`maxlen=512`) arrival-timestamp log; unparseable ones are dropped silently. The thread is never joined, matching upstream's daemon thread.

### Documented deviations from upstream's dynamic typing

Upstream is untyped Python: a malformed packet field (present but the wrong JSON type, or a non-object payload where an object is expected) triggers an unhandled exception that can crash the whole node. This port never fabricates additional *validation* of well-formed Quest data, but it does make one pragmatic, documented choice for handling protocol violations that would otherwise crash: such a field, or a valid-but-non-object top-level JSON payload, is treated as absent (the output is skipped) rather than reproducing a Python traceback. See the module docs in `src/message.rs` for the exact rule. Real Quest packets never exercise this path.

## Build and test

```sh
cargo fmt --check
cargo clippy --all-targets --all-features -- -D warnings
cargo test --all-targets
cargo build --release
```

The resulting executable is:

```text
target/release/dora-openarm-quest-receiver
```

Numerical behavior (coordinate/quaternion transforms, the One Euro filter, and the gripper mapping) is checked in `tests/` against golden values generated by running the actual upstream Python implementation under numpy 2.4.6 / scipy 1.17.1 -- the versions upstream's `pyproject.toml` requires -- not hand-derived approximations.

## Dataflow usage

```yaml
nodes:
  - id: udp-receiver
    path: dora-openarm-quest-receiver
    args: "--host 0.0.0.0 --port 5006"
    inputs:
      tick: quittable-tick-leader/tick
    outputs:
      - pose_right
      - trigger_right
      - grip_right
      - pose_left
      - pose_reference
      - trigger_left
      - grip_left
      - joystick_x_left
      - joystick_y_left
      - joystick_x_right
      - joystick_y_right
      - button_a
      - button_b
      - button_x
      - button_y
      - status
```

## Compatibility

This port targets `dora-node-api = 0.5.0` (`default-features = false`) and Arrow `54.2.1`, matching the Python `dora-rs == 0.5.0` pin in upstream's `pyproject.toml`. All coordinate/quaternion/filter/gating logic is separated from the dora and UDP-socket adapters and covered by tests; only the socket I/O and the dora event loop in `src/main.rs` are untested by design (they have no meaningful behavior beyond calling into the tested library).

## License

Apache License 2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE).
