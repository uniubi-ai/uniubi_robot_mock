# uniubi_robot_mock

**English** | [简体中文](README_zh.md)

RobotService mock runtime and simulator bridge for SDK integration development without real hardware.

## What Is Included

| Area | Path | Purpose |
|---|---|---|
| Mock runtime | `mockService/uniubi_mock/` | Self-contained x86_64 runtime package deployed to `/uniubi_mock` in a Linux VM |
| Simulator bridge | `simulation/sim2sim/` | MuJoCo bridge that exchanges motion control and robot state with the mock runtime |
| SDK Sim2Sim | `docs/sim2sim_sdk.md` | Low-level SDK client validation through MuJoCo and DDS topics |
| LowLevel policy CLI | `simulation/scripts/run_lowlevel_onnx_policy.py` | Interactive laying, standing, walking, and stop validation with the bundled ONNX policy |
| DDS helper | `simulation/scripts/setup_dds.sh` | Bind Cyclone DDS to a selected network interface for the current shell |
| Runtime guide | `docs/mock_service.md` | Deploy, start, validate, and troubleshoot the mock service |
| Simulator guide | `docs/simulation_setup.md` | Prepare MuJoCo and run the bridge |

## Minimum Loop

1. Deploy `mockService/uniubi_mock/` to `/uniubi_mock` on an x86_64 Ubuntu VM.
2. Start `robotMonitorServer`, `motionServer`, and `robotServer` in order with `sudo` and `LD_LIBRARY_PATH=/uniubi_mock/vendor/usr/lib`.
3. Configure host DDS network interface in `/uniubi_mock/etc/dds/host_config.xml` if the VM interface is not one of the defaults.
4. Start the simulator bridge from `simulation/` with `PYTHONPATH=$(pwd)`.
5. Connect SDK clients to the mock service and validate HighLevel actions or the interactive LowLevel policy loop.

See [docs/mock_service.md](docs/mock_service.md), [docs/simulation_setup.md](docs/simulation_setup.md), and [docs/sim2sim_sdk.md](docs/sim2sim_sdk.md) for the full commands.

## Supported Actions

Current mock runtime supports:

- `laying`
- `standing`
- `walking`
- `emergencyStop`
- `waveHand`
- `bipedStand`
- `handstand`
- `leftSideStand`
- `rightSideStand`

The runtime also uses the internal `recovery` action automatically after a
supported action reports a fall. It is not exposed as a user-startable action.

Biped stand actions are persistent. To return to Walking from the HighLevel
console, call `stop`, confirm that `state` reports `walking`, and then send
Walking velocity parameters.

## Compatibility Notes

- Target runtime platform: Linux `x86_64`.
- Recommended OS: Ubuntu 22.04 LTS.
- DDS: Cyclone DDS 0.10.5.
- Simulator bridge: MuJoCo is the only supported simulation backend.
- The mock runtime is for SDK integration and closed-loop simulation validation. It does not replace real robot safety validation.

## License

Original UniUbi code and documentation in this repository are licensed under the Apache License 2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE).
