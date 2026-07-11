# Support Matrix

| Dimension | Supported Values |
|---|---|
| Mock runtime platform | Linux x86_64 |
| Recommended OS | Ubuntu 22.04 LTS |
| Runtime root | `/uniubi_mock` |
| DDS | Cyclone DDS 0.10.5 |
| Host DDS domain | 42 |
| Motion DDS domain | 1 |
| Simulator backends | MuJoCo, Isaac Gym |
| Default validation backend | MuJoCo |
| MuJoCo Python | Python 3.11 environment |
| Isaac Gym Python | Python 3.8 environment |
| Supported actions | `laying`, `standing`, `walking`, `emergencyStop`, `jumpFrontflip`, `jumpSideflip`, `jumpBackflip` |
| Non-goals | Real robot safety validation, RL training, high-fidelity production physics replacement |
