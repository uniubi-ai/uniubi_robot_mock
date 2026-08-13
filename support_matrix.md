# Support Matrix

**English** | [简体中文](support_matrix_zh.md)

| Dimension | Supported Values |
|---|---|
| Mock runtime platform | Linux x86_64 |
| Recommended OS | Ubuntu 22.04 LTS |
| Runtime root | `/uniubi_mock` |
| DDS | Cyclone DDS 0.10.5 |
| Host DDS domain | 42 |
| Motion DDS domain | 1 |
| Simulator backend | MuJoCo |
| Default validation backend | MuJoCo |
| MuJoCo Python | Python 3.11 environment |
| Supported actions | `laying`, `standing`, `walking`, `emergencyStop`, `waveHand`, `bipedStand`, `handstand`, `leftSideStand`, `rightSideStand` |
| Internal safety action | `recovery` (automatically scheduled after fall detection) |
| Non-goals | Real robot safety validation, RL training, high-fidelity production physics replacement |
