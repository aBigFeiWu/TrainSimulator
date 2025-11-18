# Repository Guidelines

## Project Structure & Module Organization
Source lives in `DynamicModel/` (vehicle dynamics and planning), `Simulator/` (simulation loop and lifecycle), `TrainCommunicator/` (UDP transport and timing), and `TrajKit/` (trajectory geometry utilities). Public entry points for consumers are `TrainSimulatorAPI.cpp/h`. Resource files used at runtime sit in `Resources/` and are automatically copied into the CMake build directory. Build artifacts for local work typically go under `cmake-build-*` or a custom `build/` directory created with CMake.

## Build, Test, and Development Commands
- Configure: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug` (pick `Release` for production). CMake pulls in all `*.cpp` files from the module directories.
- Build shared library: `cmake --build build`. Outputs a `TrainSimulator` shared library and copies `Resources/` into the build tree for runtime access.
- (If you add tests) drive them with `ctest --test-dir build` after enabling tests in CMake; keep test binaries inside the build tree to avoid polluting the repo.

## Coding Style & Naming Conventions
The codebase targets C++20. Use 4-space indentation, K&R brace placement, and include standard headers before project headers when practical. Classes use PascalCase (`TrainSimulator`, `MotionPlanner`), member functions are lowerCamelCase (`setTargetVelocity`), and private members end with an underscore (`config_`). Prefer `std::clamp`, `std::unique_ptr`, and other STL utilities already in use. Keep headers lightweight and include guards aligned with existing patterns.

## Testing Guidelines
No automated tests are present yet. When adding them, prefer a single test framework (e.g., GoogleTest) and name test files `<Module>Tests.cpp` under a `tests/` directory. Include deterministic fixtures for trajectory data instead of reusing `Resources/` to isolate test cases, and aim for high-level coverage on `Simulator/` lifecycle and `DynamicModel/` planners.

## Commit & Pull Request Guidelines
If you version this repository with Git, use concise, imperative commit subjects (e.g., “Add UDP timeout handling”) and group related changes together. PRs should describe the change, list build/test commands run, and include any new configuration steps. When UI or output formatting changes, attach a short log snippet or screenshot of relevant console output. Link issues or tasks when available, and call out any runtime impacts (e.g., new required trajectories in `Resources/`).

## Data & Runtime Notes
Simulation depends on trajectory text files in `Resources/`; ensure they accompany any shared library you distribute. When running outside the build directory, set the working directory so relative resource paths resolve, or pass absolute paths via configuration to avoid missing-file errors.
