# NOMAD workspace setup for assistants

This repository is the complete source workspace. When a teammate asks you to install it, use the human-facing `README.md` as the installation procedure and put the checkout at `~/nomad_ws` unless they specify another path.

The requested scope is only: place this repository at the chosen workspace path, install the ROS 2 Jazzy and package dependencies required for the build, run `./.nomad/build.sh`, then run `./run_forest.sh`. The workspace source already contains both ROS packages; do not clone another MVSim copy or reapply the patches.

Do not run unit tests, sensor/topic checks, performance profiling, map regeneration, unrelated environment audits, or extra validations. The build result and whether the simulator starts are sufficient. If a command fails, report the failure and fix only what is necessary to complete those two steps.

Do not overwrite an existing `~/nomad_ws` or unrelated files without asking the teammate. The `build/`, `install/`, and `log/` directories are generated locally and must not be copied from another computer.
