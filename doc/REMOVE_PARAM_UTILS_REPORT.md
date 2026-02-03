To remove the legacy parameter helper package, the following steps were taken:

1. **Replaced helper usage with handwritten configs** in each node package:
   - `drivers/usb_cdc_node`: introduced a `UsbCdcConfig` with `Load`/`validate`/`summary`, removed helper includes/logging, and now logs the summary banner during startup.
   - `drivers/fake_system_node`: new `FakeSystemConfig` captures timing, topic, and initialization parameters, uses declare-get helpers, and prints its summary; the snapshot log was dropped.
   - `motion/arm_servo_node`: `ArmServoConfig` now performs range checks via `declare_get_checked`, builds the joint map manually, and exposes a summary banner; startup prints that banner.
   - `motion/arm_solve_node`: `ArmSolveConfig` keeps the same validation/summarizing logic but no longer imports or stores snapshot data; the server only logs `cfg.summary()`.
   - `system/top_hfsm_node`: all nested configs were rewritten with dedicated `Load`/`validate`/`summary` functions, the executor nodes stopped logging snapshots, and `TopHFSMNode` prints its summary after loading the config.

2. **Removed the helper package**: the folder under `src/utils/` that provided parameter helpers and its build exports were deleted, and every package dependency (`CMakeLists.txt` and `package.xml`) now omits that package.

3. **Workspace status**:
   - No code imports the helper headers or calls into its APIs.
   - Each ROS node now logs a custom `cfg.summary()` banner immediately after constructing its config.
   - The removal report documents the packages and files that were cleaned up.
