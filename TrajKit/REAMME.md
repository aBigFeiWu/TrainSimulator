D:\home\GNS9000\TrainSimulator0806\TrainSimulator\cmake-build-debug\TrainSimulator.exe
1. Configuring simulation parameters...

2. Creating simulator instance...
   姝ｅ湪浣跨敤閰嶇疆鏋勯€燭rainSimulator...
   璺嚎鍔犺浇鎴愬姛銆傛€昏窛绂伙細100627绫炽€
   1D Simulation configured. Track length: 100627m, Target station: 100627m.
   Train Constraints: MaxVel=27.7778 m/s, MaxAccel=1 m/s^2, MinAccel=-0.5 m/s^2
   Jerk Gears (Low/Mid/High): 0.25/0.5/0.75 m/s^3
   Log file 'simulation_log.dat' has been created.
   鍒楄溅鎺у埗鍣ㄥ垵濮嬪寲瀹屾垚銆
   TrainSimulator鏋勯€犳垚鍔燂紝鍥炶皟宸茶缃€
   Simulator created successfully.

--- Train Simulator Interactive Console ---
Usage: <command> [arguments]

--- Simulation Control ---
start                - Sends START command and begins simulation.
stop                 - Stops the simulation timer and sends STOP command.
status               - Print the current kinematic state of the train.
exit                 - Exit the application.

--- Mode Selection ---
mode auto            - Switch to AUTOMATIC driving mode.
mode manual          - Switch to MANUAL driving mode.

--- Manual Controls (only in manual mode) ---
3t                   - Traction Level 3 (Max Acceleration)
2t                   - Traction Level 2
1t                   - Traction Level 1
cruise               - Cruise (Maintain speed, target zero acceleration)
idle                 - Idle/Coasting (No traction/braking)
1b                   - Brake Level 1
2b                   - Brake Level 2
3b                   - Brake Level 3 (Max Deceleration)
-------------------------------------------