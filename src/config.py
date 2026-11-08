"""Competition settings: calibrate once, upload all routes before the event."""

# Calibration parameters: verify these on the assembled robot.
PWM_FREQUENCY = 1000  # PWM frequency in Hz; keep fixed during motor calibration.
LEFT_REVERSED = False  # Raise wheels and set True if positive left drive is backward.
RIGHT_REVERSED = False  # Raise wheels and set True if positive right drive is backward.
LEFT_TRIM = 1.0  # Reduce the faster wheel's multiplier after a straight-line test.
RIGHT_TRIM = 1.0  # Reduce the faster wheel's multiplier after a straight-line test.
MIN_DRIVE_PERCENT = 35  # Reliable starting duty floor; find it with calibrate.py.
PIVOT_SPEED_PERCENT = 50  # Signed equal-wheel pivot command used for angle calibration.
PIVOT_MS_PER_90 = 500  # Placeholder: measure a 90-degree pivot at the configured speed.
VSYS_REF = 4.0  # Placeholder: record battery VSYS during pivot/dead-zone calibration.
VSYS_LOW = 3.6  # Low-voltage warning threshold; verify under motor load.
ADC_REFERENCE_V = 3.3  # ADC reference voltage; refine against a meter if required.
VSYS_DIVIDER = 3.0  # Standard Pico GP29 VSYS divider ratio; do not change wiring.
VOLT_COMP_MIN = 0.85  # Minimum correction factor for voltage above the reference.
VOLT_COMP_MAX = 1.35  # Maximum correction factor; verify motor current at low battery.

# Behaviour parameters: tune before the event, not between maps.
VOLT_COMP = True  # Enable bounded voltage compensation on standard Pico battery power.
SOFT_START = True  # Ramp starts and reversals to reduce supply-current spikes.
SOFT_START_MS = 40  # Startup ramp duration; confirm a smooth start without stalls.
POWER_UPDATE_MS = 200  # Voltage averaging window; check battery sag in the serial log.
POWER_SAMPLES = 16  # ADC samples spread across a voltage window; never burst-read in control.
BRAKE_MS = 60  # Active-brake pause before stops or pivot direction changes.
INNER_MODE = "COAST"  # COAST/BRAKE/REVERSE; compare corner behaviour at low speed.
TURN_INNER_PERCENT = -25  # Signed inner command used only in REVERSE mode.
BASE_SPEED_PERCENT = 55  # Fallback base command; each selected route overrides this.
SLOW_SPEED_PERCENT = 30  # Forward command near obstacles; verify stopping distance.
SEARCH_SIDE_MAX_DEG = 110  # Maximum first-side search angle; reverse sweep is twice this.
RECENT_SIDE_MS = 150  # Lifetime of raw direction clues when no turn plan remains.
DEFAULT_SEARCH_SIDE = "RIGHT"  # Fallback direction without a route/raw direction clue.
CORNER_MIN_DEG = 25  # Minimum planned-side motion before consuming one planned corner.
FINISH_CHECK_DEG = 25  # Small left/right line check after a complete turn plan.
UTURN_MAX_DEG = 220  # Hard U-turn angle ceiling; reacquiring line ends the action earlier.
MANEUVER_TIME_MARGIN = 1.5  # Timing allowance for sensor-ended nominal manoeuvres, within hard angle caps.
OBSTACLE_IGNORE_MS = 800  # Ignore old obstacle echoes briefly after completing a U-turn.
OBSTACLE_STOP_CM = 12  # Two fresh median readings at/below this distance trigger the route action.
OBSTACLE_SLOW_CM = 25  # Slow down below this range when that route enables obstacles.
OBSTACLE_CONFIRM_READINGS = 2  # Distinct consecutive close measurement timestamps required.
NEAR_MEMORY_MS = 500  # Treat lost Echo as blocked only after a near reading within this window.
NEAR_CLEAR_MARGIN_CM = 5  # Clear a near latch only above SLOW plus this distance margin.
TURN_CLEAR_DEG = 45  # Completed cumulative pivot angle that clears the old-heading near latch.
STOP_WAIT_NO_ECHO_MS = 3000  # Fresh continuous NO_ECHO escape time in STOP_WAIT.
RECOVERY_SLOW_MS = 1000  # Slow-forward interval after the NO_ECHO deadlock escape.
MIN_VALID_CM = 2.5  # Reject blind-zone distances; invalid readings never mean clear.
CLEAR_HOLD_MS = 500  # Continuous fresh clear-reading duration before leaving STOP_WAIT.
GAP_TURN_DEG = 60  # Diagonal turn toward the offset line; calibrate on route 3.
GAP_SPEED_PERCENT = 35  # Slow straight drive across the gap; verify the diagonal intercept.
GAP_MAX_MS = 1500  # Maximum straight gap travel before a latched stop.
GAP_BIAS_MS = 1000  # Prefer the gap direction briefly if line is lost after crossing.
EDGE_INNER_RATIO = 0.55  # Gentle edge correction ratio; tune on blue/wood surfaces.
CREEP_SPEED_PERCENT = 25  # Optional ramp-gap creep command; disabled per-route by default.
ECHO_TIMEOUT_US = 30000  # Echo deadline; do not wait synchronously in the main loop.
ASYNC_TRIGGER_GAP_MS = 70  # Minimum spacing between any two ultrasonic transmissions.
MAX_READING_AGE_MS = 300  # Reject stale distance data; confirm normal sample ages below this.
SCHEDULE = ("F", "F", "L")  # Front gets two of three slots; never transmit simultaneously.
FRONT_MEDIAN_SAMPLES = 3  # Median window for valid front readings; check moving-target response.
IR_SAMPLE_MS = 5  # Keep the documented IR sampling period during speed calibration.
IR_CONFIRM_SAMPLES = 3  # Consecutive identical digital pairs required for stable line state.
IR_MISSING_MS = 100  # IR diagnostic missing-line duration; route control uses confirmed states.
IR_PRINT_MS = 200  # Standalone IR/calibration print period; avoid flooding USB serial.
DEBUG = True  # Enable transition/diagnostic logging; disable for untethered timing tests.
LOG_INTERVAL_MS = 50  # Minimum spacing between transition logs; queued logs remain ordered.
ULTRASONIC_PRINT_INTERVAL_MS = 200  # Optional serial diagnostic period in tools.
RUN_SECONDS = 2  # Standalone motor test duration, not a competition motion command.
MOTOR_TEST = False  # Standalone motor self-test switch; no motion on module import.
TEST_SPEED_PERCENT = 55  # Standalone wheel-test command; raise the wheels before running.
MANUAL_SPEED_PERCENT = 60  # Default forward/backward helper command; check wheel direction first.
MANUAL_TURN_INNER_PERCENT = 30  # Default manual gentle-turn inner command; tune on a safe surface.
MANUAL_TURN_OUTER_PERCENT = 65  # Default manual gentle-turn outer command; tune together with inner duty.
MOTOR_TEST_START_DELAY_MS = 3000  # Raised-wheel self-test countdown before applying any drive.
CALIBRATION_START_PERCENT = 20  # First raw-duty dead-zone test; inspect wheel startup.
CALIBRATION_STEP_PERCENT = 5  # Raw-duty increment between dead-zone trials.
CALIBRATION_MAX_PERCENT = 100  # Hard ceiling for raw-duty calibration trials.
STOP_SECONDS = 1  # Pause between standalone wheel tests; not used by route control.
MAIN_LOOP_MS = 1  # Cooperative main-loop sleep; control handlers never sleep.
SELECT_NEAR_CM = 8  # New VALID near reading arms a hand gesture.
SELECT_HYST_CM = 5  # Release requires >13 cm, or a new NO_ECHO reading.
SELECT_RESET_MS = 2000  # Continuous near-hand hold clears the gesture count.
SELECT_CONFIRM_MS = 2500  # Quiet released duration after gestures confirms the route.
SELECT_CLEAR_LED_MS = 1000  # LED steady-on feedback after clearing a selection.
SELECT_ERROR_MS = 2000  # Fast-blink duration for too many gestures.
SELECT_FEEDBACK_MS = 100  # Short LED pulse acknowledging each completed gesture.
SELECT_SLOW_BLINK_MS = 300  # On/off period for repeating the selected route number.
SELECT_BOOT_FLASHES = 3  # Startup quick flashes indicating that firmware is alive.
SELECT_FAST_BLINK_MS = 100  # Fast LED pulse period for startup and error feedback.
SELECT_SENSOR_FAULT_MS = 3000  # Warn if front sonar has never produced VALID data.
SELECT_SENSOR_FAULT_FLASHES = 10  # Fast LED code for unavailable front measurements.
LOW_BATTERY_LED_MS = 2000  # Steady LED warning before route confirmation flashes.
START_DELAY_MS = 5000  # Motors-off interval for placing the selected robot on START.
ROUTE_SELECT_PINS = None  # Optional (16,17) grounded jumpers only if event rules permit.
SAVE_ON_EVENTS = True  # Save progress changes; disable if calibrated flash writes exceed 40 ms.
RUN_STATE_FILE = "run_state.txt"  # Pico flash archive; keep the atomic .tmp staging file on the same volume.
RESUME_WINDOW_MS = 4000  # Startup hand-gesture window before LOST/RUNNING progress resumes automatically.
RESUME_ARM_MS = 1000  # Motors-stopped time before accepting release-then-hand resume gestures.
RESUME_COUNTDOWN_MS = 3000  # Motors-off placement countdown after a stopped-state resume gesture.
RESUME_LED_FLASH_MS = 100  # Short status LED pulse; LOST has two and FINISHED three per group.
RESUME_LED_PAUSE_MS = 600  # Dark interval between repeated saved-status LED groups.
REACQUIRE_CREEP_MS = 800  # Maximum straight probe duration between bounded reacquisition scans.
REACQUIRE_CREEP_PERCENT = 40  # Reacquisition probe command; verify low-speed travel before racing.
REACQUIRE_TRIES = 2  # Maximum creep-plus-search retry rounds after the initial search.
STATE_WRITE_TEST_SAMPLES = 5  # Atomic scratch writes averaged by calibrate.py menu 6.
STATE_BENCHMARK_FILE = "run_state_benchmark.txt"  # Separate scratch archive; never benchmark on the real progress file.
STATE_WRITE_WARN_MS = 40  # Above this measured flash latency, set SAVE_ON_EVENTS=False.

# Fixed hardware reference; keep these pins consistent with the official wiring.
LEFT_MOTOR_PINS = (2, 3, 4)
RIGHT_MOTOR_PINS = (6, 7, 8)
FRONT_ULTRASONIC_PINS = (10, 11)
LEFT_ULTRASONIC_PINS = (12, 13)
VSYS_ADC_CHANNEL = 29
USB_SENSE_GPIO = 24
LED_PIN = "LED"
LED_FALLBACK_GPIO = 25

# Route table: choose at runtime; optional creep is disabled unless calibrated.
ROUTES = {
    0: {"follow": "CENTER", "turns": "", "finish": "DEAD_END", "dead_ends": ["FINISH"], "obstacle": "STOP_WAIT", "obstacle_after_turns": 0, "base_speed": 55, "creep_on_loss_ms": 0},
    1: {"follow": "CENTER", "turns": "RRR", "finish": "AFTER_TURNS", "dead_ends": ["FINISH"], "obstacle": "NONE", "obstacle_after_turns": 0, "base_speed": 60, "creep_on_loss_ms": 0},
    2: {"follow": "CENTER", "turns": "RRRLLRRRRR", "finish": "AFTER_TURNS", "dead_ends": ["FINISH"], "obstacle": "NONE", "obstacle_after_turns": 0, "base_speed": 55, "creep_on_loss_ms": 0},
    3: {"follow": "CENTER", "turns": "LLR", "finish": "DEAD_END", "dead_ends": ["FINISH"], "obstacle": "GAP_LEFT", "obstacle_after_turns": 3, "base_speed": 55, "creep_on_loss_ms": 0},
    4: {"follow": "EDGE_RIGHT", "turns": "", "finish": "DEAD_END", "dead_ends": ["UTURN", "FINISH"], "obstacle": "DEAD_END", "obstacle_after_turns": 0, "base_speed": 50, "creep_on_loss_ms": 0},
}

# Checkpoints are intentionally empty; fill measured progress tuples before the event.
for _route in ROUTES.values():
    _route["checkpoints"] = []


def deg_to_ms(degrees):
    """Convert a calibrated angle magnitude to a nonzero millisecond bound."""
    return max(1, int(abs(degrees) * PIVOT_MS_PER_90 / 90))
