/*
package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

*/
/**
 * =====================================================================================
 *  АЛЬФА — BIOBUZZ (2026-27) TeleOp, Knightmares
 * =====================================================================================
 *
 *  WHAT THIS OPMODE DOES
 *  ---------------------
 *  Normal driver control (mecanum drive, intake, PID slide, manual flywheels and gates),
 *  plus two camera-driven automations:
 *
 *   1. SMART SHOOT (RT). The Limelight looks into the slide chamber and records the order
 *      of up to 4 balls (pollen / nectar), from the drop gate to the far end. The drop gate
 *      then opens and the balls roll toward the flywheels. Whenever the NEXT ball needs a
 *      flywheel that has not recovered from its previous shot, the high-speed FLYWHEEL gate
 *      closes and holds that ball until the wheel is back at speed.
 *
 *   2. SMART FLOWER DROP (dpad_down). Opens the flower gate for exactly long enough to drop
 *      one ball. The open time depends on which ball is dropping and which ball is behind it,
 *      both known from the camera.
 *
 *  BALL PATH THIS CODE ASSUMES
 *  ---------------------------
 *    slide chamber [b0 b1 b2 b3] -> DROP GATE ("FLOW") -> FLYWHEEL GATE ("FLYWHEEL")
 *        -> passive size sorter -> NECTAR flywheel (3.6" balls) or POLLEN flywheel (2.8" balls)
 *    b0 is the ball resting against the drop gate. The FLOWER gate is a separate exit.
 *
 *  CONTROLS (gamepad 1)
 *  --------------------
 *    Left stick / right stick X   mecanum drive / turn
 *    RB                           flip which end of the robot is "front"
 *    B / A                        spin up Nectar / Pollen flywheel (latched on)
 *    X                            stop both flywheels AND abort a running smart shoot
 *    LT / LB                      intake in / out
 *    RT                           SMART SHOOT
 *    Y                            manual flywheel gate toggle (pass / block)
 *    BACK                         manual drop-gate pulse (this was RT in the old code)
 *    dpad_up                      slide up / down (LinearSlideController PID)
 *    dpad_right                   flower gate latch open / closed
 *    dpad_down                    SMART single flower drop
 *    dpad_left                    force a camera snapshot (testing)
 *
 *  ONE LOOP, IN ORDER (see runOpMode)
 *  ----------------------------------
 *    bulk read -> buttons -> readWheels -> drive -> intake -> slide -> camera -> sequence
 *    -> gates -> write flywheel velocities -> telemetry
 *  Nothing in the loop sleeps or waits. Every timed action is a small state machine checked
 *  once per loop against an ElapsedTime, so the drivetrain never freezes (the old code's
 *  sleep() calls froze driving for up to ~1 second).
 *
 *  CHANGING THINGS
 *  ---------------
 *  Every number you are likely to tune is a "public static" field in the TUNABLES block.
 *  FTC Dashboard (http://192.168.43.1:8080/dash) shows them live under this class name.
 *  Dashboard edits take effect immediately but are LOST when the robot restarts, so copy
 *  final values back into this file.
 *
 *  See README.md in this folder for the setup checklist, tuning order and troubleshooting.
 *//*

@Config                    // exposes every public static field below to FTC Dashboard
@TeleOp(name = "АЛЬФА")    // Driver Station name kept from the original OpMode
public class LukeTeleopKnightmaresBIOBUZZ extends LinearOpMode {
    // NOTE: a public Java class must have the same name as its file. The old code was
    // "class Alpha" inside LukeTeleopKnightmaresBIOBUZZ.java, which does not compile.
    // If you rename the file on the robot, rename this class to match.

    // =================================================================================
    //  BALL CODES
    // =================================================================================
    // P and N are deliberately 0 and 1 so the same number indexes the flywheel arrays:
    // W[P] is the pollen flywheel, W[N] the nectar flywheel. That removes duplicate
    // pollen/nectar code everywhere.
    static final int P = 0;            // pollen: yellow, ~2.8", goes to the POLLEN flywheel
    static final int N = 1;            // nectar: red or blue, ~3.6", goes to the NECTAR flywheel
    static final int U = 2;            // unknown: camera unsure or dead, treated as needing BOTH wheels
    static final int E = 3;            // empty slot
    static final int MAX = 4;          // chamber capacity (the most balls the slide holds)
    static final String CH = "PN?-";   // telemetry character for each code, indexed by the code

    // =================================================================================
    //  TUNABLES (live-editable in FTC Dashboard)
    // =================================================================================

    // ---- Flywheels ----
    // Target speeds in encoder ticks per second. These MUST be speeds the motor can reach.
    // The old values (100000) are far above any motor's top speed: the hub just ran full power
    // and the "is the wheel up to speed?" check could never pass. A bare goBILDA 6000 rpm
    // motor tops out near 2800 t/s, so 2600 leaves a little headroom. Raise or lower until
    // shots land AND telemetry shows "RDY".
    public static double NECTAR_VEL = 2600;
    public static double POLLEN_VEL = 2600;
    // Intake speed. 7500 is above top speed, so it runs at full power, exactly as the old code
    // did. Left alone on purpose because intake speed isn't used for any timing.
    public static double INTAKE_VEL = 7500;
    // A wheel is "ready" when its measured speed is within READY_TOL of target (0.05 = 5%).
    public static double READY_TOL = 0.05;
    // A shot is counted when a ready wheel drops below (1 - DIP_FRAC) of target (8% dip).
    // If the "fired" telemetry count misses shots, lower this; if it counts extras, raise it.
    public static double DIP_FRAC = 0.08;
    // true  = holds end as soon as the wheel has fired and recovered (fastest, needs good dips).
    // false = holds use only the fixed PP_HOLD_MS / NN_HOLD_MS times (use if speed reads are noisy).
    public static boolean USE_VEL_READY = true;
    // true  = track recovery per flywheel, so P-N-P also waits if the pollen wheel isn't ready.
    // false = only hold two SAME-type balls in a row (the original spec). See README.
    public static boolean SPACE_ALL = true;
    // true = open the drop gate during spin-up with the flywheel gate CLOSED, so the balls are
    // already waiting at the flywheel gate when the wheels are ready (~250 ms faster). Only use
    // it if the flywheel gate can separate touching balls. Off by default.
    public static boolean PRESTAGE = false;
    // true = turn both flywheels off after a smart shoot. false = leave them spinning for the next cycle.
    public static boolean SPIN_DOWN_AFTER = false;
    // true if the flower gate is at the FAR end of the slide (opposite the drop gate).
    // This decides which ball in the order is "the one that drops".
    public static boolean FLOWER_FROM_FAR_END = false;

    // ---- Servo positions ----
    // These servos are positional servos configured as CRServo, so setPower(p) acts as a
    // position: -1 to +1 maps to the servo's full range. Kept that way so the robot
    // configuration file does not need to change.
    public static double FW_PASS = 0.35;    // flywheel gate OPEN  (old "posUp")
    public static double FW_BLOCK = -0.95;  // flywheel gate CLOSED (old "posDown"). Swap the two if reversed.
    public static double flowerClose = 0;   // flower gate closed (unchanged from old code)
    public static double flowerOpen = 0.75; // flower gate open   (unchanged)
    public static double stopFlow = -0.4;   // drop gate closed, holds the balls on the slide (unchanged)
    public static double letFlow = 0;       // drop gate open, balls roll off (unchanged)
    public static double unlockpos = 0.15;  // UNLOCK servo, set once at init (unchanged)

    // ---- Camera ----
    public static int CAM_MODE = 0;            // 0 = limelight_ball_indexer.py colour script, 1 = neural detector
    public static int CAM_PIPELINE = 0;        // Limelight pipeline slot holding that script or model
    public static int SNAP_FRAMES = 4;         // frames voted on per snapshot (more = steadier, slower)
    public static double SNAP_TIMEOUT_MS = 500;    // finish a snapshot with whatever frames arrived by then
    public static double CAM_STALE_MS = 150;       // ignore frames older than this (camera lag or freeze)
    public static double INDEX_SETTLE_MS = 300;    // wait this long after the chamber changes before auto-indexing
    public static double DET_CONF = 0.45;          // detector mode: drop detections below this confidence
    public static boolean DET_AXIS_X = true;       // detector mode: chamber runs left-right (true) or up-down (false) in the image
    public static boolean DET_GATE_NEG = true;     // detector mode: drop gate is at negative tx (left) / ty (bottom)

    // ---- Smart shoot timing (milliseconds) ----
    public static double SPIN_MIN_MS = 150;        // always spin at least this long, so speed readings settle
    public static double SPIN_TIMEOUT_MS = 2000;   // if wheels never read "ready", shoot anyway after this
    public static double FIRST_ARRIVAL_MS = 250;   // drop gate opens -> ball 0 reaches the flywheel gate
    public static double POLLEN_CLEAR_MS = 90;     // a rolling pollen takes this long to pass the flywheel gate
    public static double NECTAR_CLEAR_MS = 115;    // nectar is bigger (3.6" vs 2.8"), so it takes longer.
                                                   //   These two set WHEN the gate can close behind a ball,
                                                   //   which is why stopping a 2nd nectar differs from a 2nd pollen.
    public static double FROM_REST_EXTRA_MS = 80;  // a ball released from a stop at the gate is slower than a rolling one
    public static double PP_HOLD_MS = 350;         // pollen wheel recovery cap: max gap needed between two pollen
    public static double NN_HOLD_MS = 450;         // nectar wheel recovery cap: heavier ball, bigger dip, longer recovery
    public static double GATE_MIN_MS = 60;         // min time the flywheel gate stays shut (servo must finish moving)
    public static double HOLD_MAX_MS = 1000;       // never hold one ball longer than this, whatever the wheel says
    public static double FINAL_FIRE_MS = 400;      // after the last ball passes the gate, time for it to leave its wheel
    public static double FLOW_PULSE_MS = 400;      // manual BACK drop-gate pulse length (old "flowtime")

    // ---- Smart flower drop timing (milliseconds) ----
    // Open time = base time for the DROPPING ball + trim for the ball BEHIND it.
    public static double FLOWER_NECTAR_MS = 550;           // dropping a nectar (old flowerOnceTime)
    public static double FLOWER_POLLEN_MS = 430;           // dropping a pollen: smaller, clears the gate sooner
    public static double FLOWER_UNKNOWN_MS = 550;          // no camera data: use the safe (nectar) time
    public static double FLOWER_NEXT_NECTAR_TRIM_MS = 0;   // a big nectar behind can't slip under a half-closed gate
    public static double FLOWER_NEXT_POLLEN_TRIM_MS = -40; // a small pollen behind CAN, so close earlier
    public static double FLOWER_LAST_EXTRA_MS = 100;       // nothing behind: hold open longer for a clean drop

    // =================================================================================
    //  HARDWARE
    // =================================================================================
    // Config names and ports match the original OpMode (port numbers in comments).
    private final DcMotor[] D = new DcMotor[4];      // drive: [0]=LB 0C, [1]=LF 1C, [2]=RB 2C, [3]=RF 3C
    private final DcMotorEx[] W = new DcMotorEx[2];  // flywheels: [P]="POLLEN" 1E, [N]="NECTAR" 0E
    private DcMotorEx INTAKE;                        // "INTAKE" 3E
    private CRServo FLOW;                            // "FLOW" 1S     drop gate at the end of the slide chamber
    private CRServo FLOWER;                          // "FLOWER" 2S   flower gate
    private CRServo FWGATE;                          // "FLYWHEEL" 5S high-speed gate before the sorter
                                                     // ("UNLOCK" 4S is set once in init; 3S was the unused CamServo)
    private Limelight3A cam;                         // "limelight" (null if not in the config: robot still runs, blind)
    private List<LynxModule> hubs;                   // Control Hub + Expansion Hub, for bulk reads
    private LinearSlideController slide;             // existing slide PID class, drives "LS" 2E (file not in this folder)

    // =================================================================================
    //  WRITE CACHE
    // =================================================================================
    // Every motor or servo command is a USB/RS-485 message to a hub, and on REV hardware
    // those messages are the slowest part of a loop. out()/vel() remember the last value
    // sent to each device and skip the message if nothing changed. A robot sitting still
    // with the gates closed sends almost nothing.
    private static final int O_FW = 4;      // slot numbers in last[]: 0-3 are the drive motors
    private static final int O_FLOWER = 5;
    private static final int O_FLOW = 6;
    private static final int O_IN = 7;
    private static final int O_W = 8;       // 8 = pollen flywheel, 9 = nectar flywheel (O_W + P or N)
    private final double[] last = new double[10];  // filled with NaN at init so the first write always goes out

    */
/** Set power (or servo position) only if it differs from what was last sent. *//*

    private void out(DcMotorSimple d, int id, double v) {
        if (v != last[id]) { d.setPower(v); last[id] = v; }   // NaN != anything, so the first call always sends
    }

    */
/** Set velocity (ticks/s) only if it differs from what was last sent. *//*

    private void vel(DcMotorEx m, int id, double v) {
        if (v != last[id]) { m.setVelocity(v); last[id] = v; }
    }

    // =================================================================================
    //  BUTTONS (rising-edge detection)
    // =================================================================================
    // The old code checked "is the button held?" every loop, so a toggle like RB flipped
    // dozens of times per press, and sleep() was used to hide that. Here every button is
    // one bit in an int. Comparing this loop's bits to last loop's bits gives a clean
    // "just pressed" signal with no sleeping.
    private static final int B_A = 1, B_B = 2, B_X = 4, B_Y = 8, B_RB = 16, B_DD = 32, B_DR = 64,
            B_DL = 128, B_BACK = 256, B_RT = 512;
    private int bNow;   // buttons held this loop
    private int bPrev;  // buttons held last loop

    */
/** Snapshot every button we use into one int. Called once per loop. *//*

    private void readButtons(Gamepad g) {
        bPrev = bNow;
        bNow = (g.a ? B_A : 0) | (g.b ? B_B : 0) | (g.x ? B_X : 0) | (g.y ? B_Y : 0)
                | (g.right_bumper ? B_RB : 0) | (g.dpad_down ? B_DD : 0) | (g.dpad_right ? B_DR : 0)
                | (g.dpad_left ? B_DL : 0) | (g.back ? B_BACK : 0)
                | (g.right_trigger > 0.3 ? B_RT : 0);   // trigger counts as "pressed" past 30% travel
    }

    */
/** True only on the single loop where button b goes from up to down. *//*

    private boolean pressed(int b) { return (bNow & b) != 0 && (bPrev & b) == 0; }

    // ---- Misc loop state ----
    private final ElapsedTime clock = new ElapsedTime();   // free-running ms clock for per-wheel timestamps
    private final ElapsedTime loopT = new ElapsedTime();   // measures loop time (shown in telemetry)
    private final ElapsedTime telT = new ElapsedTime();    // limits telemetry to 10 Hz
    private double loopMs;                                  // last loop's duration
    private boolean flip;                                   // drive direction flipped (old DRIVE_STATE)
    private boolean fwBlocked;                              // current flywheel gate state, for the Y toggle
    private final double[] pw = new double[4];              // drive powers LB LF RB RF (reused, no garbage)

    // =================================================================================
    //  OPMODE ENTRY
    // =================================================================================
    @Override
    public void runOpMode() {
        Arrays.fill(last, Double.NaN);   // write cache starts "unknown" so every device gets its first command

        // Telemetry goes to both the Driver Station and FTC Dashboard.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);   // 10 updates/s is plenty to read; faster wastes Wi-Fi

        // ---- Drive motors ----
        String[] dn = {"LB", "LF", "RB", "RF"};
        for (int i = 0; i < 4; i++) {
            D[i] = hardwareMap.get(DcMotor.class, dn[i]);
            // RUN_USING_ENCODER kept from the old code: the hub's speed control makes all four
            // wheels match, so the robot drives straighter. Change to RUN_WITHOUT_ENCODER if the
            // drive feels slow to respond. (The old encoder reset is gone: TeleOp never reads
            // drive positions.)
            D[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Set explicitly so the robot stops when the sticks are released, whatever the SDK default.
            D[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        D[0].setDirection(DcMotor.Direction.REVERSE);   // LB reversed (as before)
        D[1].setDirection(DcMotor.Direction.REVERSE);   // LF reversed (as before)

        // ---- Flywheels and intake ----
        W[P] = hardwareMap.get(DcMotorEx.class, "POLLEN");
        W[N] = hardwareMap.get(DcMotorEx.class, "NECTAR");
        INTAKE = hardwareMap.get(DcMotorEx.class, "INTAKE");
        W[P].setDirection(DcMotor.Direction.REVERSE);    // as before
        INTAKE.setDirection(DcMotor.Direction.REVERSE);  // as before
        for (DcMotorEx m : new DcMotorEx[]{W[P], W[N], INTAKE}) {
            // setVelocity() only works in RUN_USING_ENCODER; set it explicitly instead of relying on the default.
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // FLOAT = coast down when turned off instead of braking hard. Kinder to flywheel
            // gearboxes and avoids a current spike. (Change to BRAKE if the intake must stop dead.)
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        // ---- Slide motor (owned by LinearSlideController) ----
        DcMotor LS = hardwareMap.get(DcMotor.class, "LS");
        LS.setDirection(DcMotor.Direction.REVERSE);      // as before

        // ---- Servos ----
        FLOW = hardwareMap.get(CRServo.class, "FLOW");
        FLOWER = hardwareMap.get(CRServo.class, "FLOWER");
        FWGATE = hardwareMap.get(CRServo.class, "FLYWHEEL");
        FLOWER.setDirection(DcMotorSimple.Direction.REVERSE);       // as before
        hardwareMap.get(CRServo.class, "UNLOCK").setPower(unlockpos); // set once, never moved again (as before)
        fw(false);                          // flywheel gate open (same start position as the old posUp)
        out(FLOWER, O_FLOWER, flowerClose); // flower gate closed
        out(FLOW, O_FLOW, stopFlow);        // drop gate closed, balls held on the slide

        // ---- Slide PID ----
        slide = new LinearSlideController(LS);   // slide must be fully DOWN at init (it zeroes here)
        slide.init();

        // ---- Camera ----
        // tryGet returns null instead of throwing if "limelight" isn't in the robot config,
        // so a missing or unplugged camera can't crash the OpMode. Smart shoot then runs blind.
        cam = hardwareMap.tryGet(Limelight3A.class, "limelight");
        if (cam != null) {
            cam.setPollRateHz(100);           // the SDK asks the Limelight for a result 100x/s
            cam.pipelineSwitch(CAM_PIPELINE); // select the ball-indexer pipeline
            cam.start();                      // start the background polling
        }

        // ---- Bulk reads ----
        // MANUAL caching: the first sensor read each loop fetches EVERY encoder and velocity
        // on a hub in one message, and later reads in that loop use the cached copy. Without
        // this, each getVelocity() is its own round trip (~2 ms each).
        // Switched on AFTER slide.init() because init may read the slide encoder in a loop,
        // and with MANUAL caching that read would never refresh until clearBulkCache().
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // "Previous ball on this wheel" timestamps start far in the past, so the first ball
        // to each wheel never waits.
        Arrays.fill(lastClr, -1e9);

        telemetry.addLine("Привет!");
        telemetry.addData("Camera", cam == null ? "NOT FOUND - blind mode" : "Limelight OK");
        telemetry.update();

        waitForStart();
        slide.start();
        dirty();   // schedule an index of the preloaded balls once the robot settles

        // =============================== MAIN LOOP ===============================
        while (opModeIsActive()) {
            loopMs = loopT.milliseconds();   // how long the previous loop took
            loopT.reset();
            for (LynxModule h : hubs) h.clearBulkCache();   // throw away last loop's sensor data

            Gamepad g = gamepad1;
            readButtons(g);

            readWheels();          // flywheel speeds, A/B/X buttons, shot detection
            drive(g);              // mecanum drive
            intake(g);             // intake motor (marks the chamber as changed while running)
            slide.update(g.dpad_up); // slide PID (unchanged call from the old code)
            camera();              // snapshots: manual, automatic, or requested by shoot/flower
            sequence();            // smart shoot state machine (RT)
            gates();               // manual gates + smart flower drop

            // Written every loop (the cache skips repeats), so a Dashboard speed change applies live.
            for (int w = 0; w < 2; w++) vel(W[w], O_W + w, on[w] ? tgt(w) : 0);

            telem();
        }
        if (cam != null) cam.stop();   // stop Limelight polling when the OpMode ends
    }

    // =================================================================================
    //  DRIVE
    // =================================================================================
    */
/** Mecanum drive. Same math and sign conventions as the old Drive()/MecanumDrive(). *//*

    private void drive(Gamepad g) {
        if (pressed(B_RB)) flip = !flip;   // edge-detected: one press = one flip (old code flipped every loop)

        // Flipping the front negates forward and strafe; turning stays the same.
        // flip == false reproduces the old MecanumDrive(left_stick_y, -left_stick_x, right_stick_x).
        double s = flip ? -1 : 1;
        double f = s * g.left_stick_y;    // forward
        double x = -s * g.left_stick_x;   // strafe
        double r = g.right_stick_x;       // turn

        pw[0] = f - x + r;   // LB
        pw[1] = f + x + r;   // LF
        pw[2] = f + x - r;   // RB
        pw[3] = f - x - r;   // RF

        // Normalise: if any wheel would exceed 1.0, scale all four down together so the
        // direction of travel is kept. Dividing by max(1, ...) leaves small inputs untouched.
        double m = Math.max(1, Math.max(Math.max(Math.abs(pw[0]), Math.abs(pw[1])),
                Math.max(Math.abs(pw[2]), Math.abs(pw[3]))));
        for (int i = 0; i < 4; i++) out(D[i], i, pw[i] / m);
    }

    // =================================================================================
    //  INTAKE
    // =================================================================================
    private boolean intakeIdle = true;   // false while the intake runs; auto-index waits for true

    private void intake(Gamepad g) {
        // 0.1 trigger deadband stops a slightly worn trigger from running the intake (old code used > 0).
        double v = g.left_trigger > 0.1 ? INTAKE_VEL : g.left_bumper ? -INTAKE_VEL : 0;
        vel(INTAKE, O_IN, v);
        intakeIdle = v == 0;
        // Any intake movement can change what's in the chamber, so the stored order is no longer
        // trusted. dirty() is called every loop the intake runs, which keeps restarting the settle
        // timer, so the auto-index happens INDEX_SETTLE_MS after the intake STOPS.
        if (!intakeIdle) dirty();
    }

    // =================================================================================
    //  FLYWHEELS + SHOT DETECTION
    // =================================================================================
    private final boolean[] on = new boolean[2];     // is this wheel commanded on? (indexed by P / N)
    private final boolean[] armed = new boolean[2];  // was this wheel at speed (ready to notice a dip)?
    private final double[] wv = new double[2];       // measured speed this loop, ticks/s
    private final int[] shotCnt = new int[2];        // shots detected per wheel since start (telemetry "fired")

    */
/** Target speed for wheel w. Read from the tunables every call, so Dashboard edits apply immediately. *//*

    private double tgt(int w) { return w == N ? NECTAR_VEL : POLLEN_VEL; }

    */
/** Wheel w is on AND within READY_TOL of its target. *//*

    private boolean near(int w) {
        double t = tgt(w);
        return on[w] && Math.abs(wv[w] - t) <= t * READY_TOL;
    }

    */
/**
     * Is the wheel a ball of this type needs ready? An unknown ball could go to either wheel,
     * so both must be ready. With USE_VEL_READY off this always says yes (timing only).
     *//*

    private boolean ready(int type) {
        if (!USE_VEL_READY) return true;
        return type == U ? near(P) && near(N) : near(type);
    }

    */
/** Flywheel buttons and shot detection. Runs before the sequence so it sees fresh shot counts. *//*

    private void readWheels() {
        // A/B latch a wheel on until X. Same behaviour as the old code (its setVelocity stayed
        // in effect after release); the state is now tracked so the smart shoot knows it.
        if (pressed(B_B)) on[N] = true;   // B: nectar wheel on
        if (pressed(B_A)) on[P] = true;   // A: pollen wheel on
        if (pressed(B_X)) {               // X = everything off, and abort any smart shoot
            on[P] = on[N] = false;
            if (seq != S_IDLE) end("ABORTED (X)", true);
        }

        for (int w = 0; w < 2; w++) {
            wv[w] = W[w].getVelocity();   // comes from this loop's bulk read, no extra hub message
            // Shot detection is a 2-state latch:
            //   wheel reaches the ready band          -> armed
            //   armed wheel dips below the dip limit  -> one shot counted, disarm
            // Re-arming only once the wheel is back in the band means one ball = one count,
            // and a wheel still spinning up never counts a fake shot.
            if (!on[w]) armed[w] = false;
            else if (near(w)) armed[w] = true;
            else if (armed[w] && wv[w] < tgt(w) * (1 - DIP_FRAC)) {
                armed[w] = false;
                shotCnt[w]++;
            }
        }
    }

    // =================================================================================
    //  CAMERA INDEXING
    // =================================================================================
    // Result of the last good snapshot. q[0] = ball touching the drop gate, q[qn-1] = far end.
    private final int[] q = new int[MAX];
    private int qn;                       // number of balls in q
    private boolean snapValid;            // q matches the chamber (nothing has moved since it was taken)
    private boolean camOk;                // last snapshot received at least one frame
    private boolean snapping;             // a snapshot is collecting frames right now
    private boolean autoIdx;              // a snapshot is scheduled for when the chamber settles

    // Voting: several frames are collected and voted on, so one bad frame (glare, motion
    // blur, a ball mid-roll) can't wreck the order.
    private final int[][] votes = new int[MAX][2];   // votes[slot][P or N]
    private final int[] cntVotes = new int[MAX + 1]; // cntVotes[k] = frames that saw k balls
    private int frames;                              // good frames in this snapshot
    private final int[] fT = new int[MAX];           // scratch: one frame's ball types, gate first
    private final double[] fPos = new double[MAX];   // scratch: one frame's ball positions (detector mode)
    private LLResult lastRes;                        // last frame used, so the same frame is never counted twice
    private final ElapsedTime snapT = new ElapsedTime();    // time since this snapshot started
    private final ElapsedTime settleT = new ElapsedTime();  // time since the chamber last changed

    */
/** The chamber contents changed: stop trusting q, and re-index once things settle. *//*

    private void dirty() {
        snapValid = false;
        autoIdx = true;
        settleT.reset();
    }

    */
/** Begin collecting frames. Does nothing if a snapshot is already running. *//*

    private void snap() {
        if (snapping) return;
        for (int[] v : votes) Arrays.fill(v, 0);
        Arrays.fill(cntVotes, 0);
        frames = 0;
        snapping = true;
        snapT.reset();
    }

    */
/**
     * Called every loop. Starts snapshots (dpad_left, or automatically once the chamber has
     * settled) and feeds new camera frames into the vote. Only reads the camera while a
     * snapshot is running, so it costs nothing the rest of the time.
     *//*

    private void camera() {
        if (pressed(B_DL)) snap();   // manual test snapshot

        // Auto-index: only once the intake has stopped, no smart shoot is running, and the
        // chamber has been still for INDEX_SETTLE_MS (balls stop rolling). Then RT can
        // usually fire straight away with no camera wait.
        if (autoIdx && intakeIdle && seq == S_IDLE && settleT.milliseconds() > INDEX_SETTLE_MS) {
            autoIdx = false;
            snap();
        }
        if (!snapping) return;

        LLResult r = cam == null ? null : cam.getLatestResult();
        // getLatestResult() returns the same object until the Limelight sends a new frame, so
        // "r != lastRes" means "this is a new frame". Old frames (camera lag or freeze) are skipped.
        if (r != null && r != lastRes && r.getStaleness() < CAM_STALE_MS) {
            lastRes = r;
            int c = CAM_MODE == 1 ? readDetector(r) : readPython(r);   // fills fT[0..c-1], or -1 if unusable
            if (c >= 0) {
                cntVotes[c]++;                                       // vote for "there are c balls"
                for (int i = 0; i < c; i++) if (fT[i] != U) votes[i][fT[i]]++;   // vote per slot
                frames++;
            }
        }

        // Finish when enough frames have been collected, time runs out, or there is no camera.
        if (cam == null || frames >= SNAP_FRAMES || snapT.milliseconds() > SNAP_TIMEOUT_MS) {
            snapping = false;
            camOk = frames > 0;   // zero frames = camera dead/unplugged/wrong pipeline -> blind mode
            // Ball count = the count seen most often (ties go to the smaller count).
            int c = 0;
            if (camOk) for (int k = 1; k <= MAX; k++) if (cntVotes[k] > cntVotes[c]) c = k;
            // Each slot = whichever type got more votes. A tie means we genuinely don't know: U.
            for (int i = 0; i < MAX; i++) {
                int p = votes[i][P], n = votes[i][N];
                q[i] = i >= c ? E : p > n ? P : n > p ? N : U;
            }
            qn = c;
            snapValid = camOk;
        }
    }

    */
/**
     * CAM_MODE 0: read the output of limelight_ball_indexer.py.
     * Format: [1, count, s0, s1, s2, s3, ...]. The leading 1 is a heartbeat proving the
     * script ran. Slot codes: 1 = pollen, 2 = red nectar, 3 = blue nectar.
     * Returns the ball count, or -1 if the frame is unusable.
     *//*

    private int readPython(LLResult r) {
        double[] o = r.getPythonOutput();
        if (o == null || o.length < 2 + MAX || o[0] != 1) return -1;   // not our script, or no heartbeat
        int c = Math.max(0, Math.min(MAX, (int) o[1]));                // clamp count to 0..4
        for (int i = 0; i < c; i++) {
            int v = (int) o[2 + i];
            fT[i] = v == 1 ? P : (v == 2 || v == 3) ? N : U;   // red and blue nectar use the same wheel
        }
        return c;
    }

    */
/**
     * CAM_MODE 1: read a Limelight neural DETECTOR pipeline (e.g. a community BIOBUZZ model
     * with classes "Pollen", "RedNectar", "BlueNectar"). Keeps confident detections whose
     * class name contains "pollen" or "nectar", sorted by position along the chamber, and
     * keeps the 4 closest to the gate. The insertion sort uses fixed arrays: no garbage.
     *//*

    private int readDetector(LLResult r) {
        List<LLResultTypes.DetectorResult> ds = r.getDetectorResults();
        if (ds == null) return -1;
        int c = 0;   // detections kept so far (sorted, gate first)
        for (LLResultTypes.DetectorResult d : ds) {
            if (d.getConfidence() < DET_CONF) continue;
            String nm = d.getClassName().toLowerCase();
            int t = nm.contains("pollen") ? P : nm.contains("nectar") ? N : -1;
            // Position along the chamber. Flip the sign if the gate is on the positive side,
            // so "smaller number = closer to gate" always holds.
            double pos = (DET_AXIS_X ? d.getTargetXDegrees() : d.getTargetYDegrees()) * (DET_GATE_NEG ? 1 : -1);
            // Skip non-balls, and skip a ball farther than all 4 we already have.
            if (t < 0 || (c == MAX && pos >= fPos[MAX - 1])) continue;
            // Insertion sort: shift farther balls back one place and drop this one in.
            // When the list is full, the farthest ball falls off the end.
            int i = Math.min(c, MAX - 1);
            for (; i > 0 && fPos[i - 1] > pos; i--) {
                fPos[i] = fPos[i - 1];
                fT[i] = fT[i - 1];
            }
            fPos[i] = pos;
            fT[i] = t;
            if (c < MAX) c++;
        }
        return c;
    }

    // =================================================================================
    //  SMART SHOOT STATE MACHINE (RT)
    // =================================================================================
    //  IDLE --RT--> INDEX --> SPIN-UP --> FEED <--> HOLD
    //                                      |
    //                                      +--last ball--> FINISH --> IDLE
    //  X aborts from any step.
    private static final int S_IDLE = 0;   // waiting for RT
    private static final int S_SNAP = 1;   // waiting for the camera order
    private static final int S_SPIN = 2;   // waiting for the needed flywheels to reach speed
    private static final int S_FEED = 3;   // a ball is passing the flywheel gate
    private static final int S_HOLD = 4;   // flywheel gate closed, next ball waits for its wheel
    private static final int S_DONE = 5;   // last ball through the gate, waiting for it to fire
    private static final String[] SN = {"IDLE", "INDEX", "SPIN-UP", "FEED", "HOLD", "FINISH"};

    private int seq = S_IDLE;                         // current step
    private int si;                                   // index (in shots[]) of the ball being fed or held
    private int shotN;                                // number of balls in this sequence
    private final int[] shots = new int[MAX];         // this sequence's ball order (a copy of q)
    private final int[] relMark = new int[2];         // per wheel: shotCnt when its latest ball was released
    private final double[] lastClr = new double[2];   // per wheel: clock time its latest ball passed the gate
    private final boolean[] wasOn = new boolean[2];   // wheel states before RT, restored after indexing
    private boolean fromRest;                         // current ball started from a stop at the gate
    private final ElapsedTime seqT = new ElapsedTime();  // time in the current step
    private String msg = "ready";                     // last result, shown in telemetry

    */
/** Change step and restart the step timer. *//*

    private void go(int s) { seq = s; seqT.reset(); }

    */
/** Open or close the flywheel gate. *//*

    private void fw(boolean block) {
        fwBlocked = block;
        out(FWGATE, O_FW, block ? FW_BLOCK : FW_PASS);
    }

    */
/** How long a ball of this type takes to fully pass the flywheel gate. Unknown uses the bigger (nectar) time. *//*

    private double clearTime(int type) { return type == P ? POLLEN_CLEAR_MS : NECTAR_CLEAR_MS; }

    */
/** Longest wait a wheel can need between two of its own balls. *//*

    private double holdCap(int w) { return w == P ? PP_HOLD_MS : NN_HOLD_MS; }

    */
/**
     * Release ball si toward the sorter. For each wheel it will use, record the current
     * shot count, so a later dip on that wheel can be recognised as THIS ball firing.
     *//*

    private void feed(boolean rest) {
        fromRest = rest;
        for (int w = 0; w < 2; w++) if (shots[si] == U || shots[si] == w) relMark[w] = shotCnt[w];
        go(S_FEED);
    }

    */
/**
     * THE KEY RULE: may ball si enter the sorter right now?
     * For each wheel the ball needs (both for unknown):
     *   spaced = that wheel's previous ball passed the gate at least PP_HOLD_MS / NN_HOLD_MS ago
     *   shot   = that wheel has fired since its previous ball was released (speed dip seen)
     *   speed check on:  OK when (spaced OR shot) AND the wheel is back within READY_TOL
     *   speed check off: OK when spaced
     * Two of the same type in a row are therefore always held: the first hasn't reached its
     * wheel yet, so it hasn't fired and isn't spaced. The hold ends the moment the wheel has
     * fired AND recovered, usually before the fixed cap, which is the "artificial delay"
     * kept as short as the flywheel physically allows.
     *//*

    private boolean clearToFeed() {
        int t = shots[si];
        // SPACE_ALL off: a ball after a DIFFERENT type only needs its wheel at speed.
        if (!SPACE_ALL && shots[si - 1] != t && t != U && shots[si - 1] != U) return ready(t);
        double now = clock.milliseconds();
        for (int w = 0; w < 2; w++) {
            if (t != U && t != w) continue;   // this ball doesn't use wheel w
            boolean spaced = now - lastClr[w] >= holdCap(w);
            boolean shot = shotCnt[w] > relMark[w];
            if (USE_VEL_READY ? !((spaced || shot) && near(w)) : !spaced) return false;
        }
        return true;
    }

    */
/** Smart shoot, advanced one step per loop. Never blocks. *//*

    private void sequence() {
        double t = seqT.milliseconds();   // time spent in the current step
        switch (seq) {

            case S_IDLE:
                // Start on RT, but not while a flower drop or manual drop pulse is moving balls.
                if (!pressed(B_RT) || flowerPulse || flowerPending || flowPulse) return;
                flowerLatched = false;
                out(FLOWER, O_FLOWER, flowerClose);   // an open flower gate would dump the chamber into the flower
                wasOn[P] = on[P];                     // remember what the driver had on
                wasOn[N] = on[N];
                if (!snapValid) {
                    // Need a new order. Spin BOTH wheels while the camera works: spin-up takes far
                    // longer than a snapshot, so the snapshot costs no time.
                    snap();
                    on[P] = on[N] = true;
                }
                msg = "indexing";
                go(S_SNAP);
                return;

            case S_SNAP:
                if (snapping) return;   // wait for the camera
                on[P] = wasOn[P];       // undo the "spin both" from above...
                on[N] = wasOn[N];
                if (snapValid && qn == 0) {
                    // Working camera sees nothing: don't run gates for no reason. If you know
                    // better, use BACK (manual drop pulse) and A/B.
                    end("chamber empty (camera)", false);
                    return;
                }
                if (snapValid) {
                    shotN = qn;
                    System.arraycopy(q, 0, shots, 0, qn);   // copy, so camera updates can't change a running sequence
                } else {
                    // BLIND MODE (camera failed): assume a full chamber of unknowns. Every unknown
                    // needs both wheels, so every gap gets the full hold. Slow but safe.
                    shotN = MAX;
                    Arrays.fill(shots, U);
                }
                // ...then turn on only the wheels these balls need (plus any the driver had on).
                for (int i = 0; i < shotN; i++) {
                    if (shots[i] != N) on[P] = true;   // pollen or unknown needs the pollen wheel
                    if (shots[i] != P) on[N] = true;   // nectar or unknown needs the nectar wheel
                }
                msg = snapValid ? "firing " + shotN : "BLIND (no camera)";
                if (PRESTAGE) {
                    fw(true);                     // flywheel gate shut...
                    out(FLOW, O_FLOW, letFlow);   // ...drop gate open: balls roll up and wait at the gate
                }
                go(S_SPIN);
                return;

            case S_SPIN: {
                // Go when every needed wheel is ready (and, with PRESTAGE, the balls have had
                // time to reach the gate), or after SPIN_TIMEOUT_MS whatever the wheels say.
                boolean ok = t >= SPIN_MIN_MS && (!PRESTAGE || t >= FIRST_ARRIVAL_MS);
                for (int i = 0; i < shotN; i++) ok &= ready(shots[i]);
                if (!ok && t < SPIN_TIMEOUT_MS) return;
                out(FLOW, O_FLOW, letFlow);   // drop gate open (no-op if PRESTAGE already opened it)
                fw(false);                    // flywheel gate open
                si = 0;
                feed(PRESTAGE);               // with PRESTAGE ball 0 starts from rest at the gate
                return;
            }

            case S_FEED: {
                // How long until ball si has fully passed the flywheel gate?
                double win = clearTime(shots[si])
                        + (si == 0 && !PRESTAGE ? FIRST_ARRIVAL_MS : 0)   // first ball rolls from the slide
                        + (fromRest ? FROM_REST_EXTRA_MS : 0);            // held balls start slower
                if (t < win) return;
                // Ball si has passed the gate: timestamp it against its wheel(s) for spacing.
                for (int w = 0; w < 2; w++) if (shots[si] == U || shots[si] == w) lastClr[w] = clock.milliseconds();
                if (++si >= shotN) { go(S_DONE); return; }   // that was the last ball
                if (clearToFeed()) { feed(false); return; }  // next ball's wheel is ready: gate stays open, it rolls straight in
                fw(true);                                    // otherwise close the gate before it reaches the sorter...
                go(S_HOLD);                                  // ...and wait
                return;
            }

            case S_HOLD:
                // Reopen once the servo has had time to physically close AND the wheel is ready,
                // or after HOLD_MAX_MS as a last resort (e.g. a wheel that never reaches target).
                if ((t >= GATE_MIN_MS && clearToFeed()) || t >= HOLD_MAX_MS) {
                    fw(false);
                    feed(true);   // this ball starts from rest
                }
                return;

            case S_DONE:
                if (t >= FINAL_FIRE_MS) end("done, fed " + shotN, SPIN_DOWN_AFTER);
                return;
        }
    }

    */
/** Stop a sequence (finished, empty, or aborted) and put the gates back. *//*

    private void end(String m, boolean stopWheels) {
        out(FLOW, O_FLOW, stopFlow);   // drop gate closed
        fw(false);                     // flywheel gate back to its normal open position
        if (stopWheels) on[P] = on[N] = false;
        // If the drop gate might have opened (spin-up or later), the chamber has changed:
        // clear the order and re-index. A ball left in the chamber shows up in telemetry.
        if (seq >= S_SPIN) {
            qn = 0;
            Arrays.fill(q, E);
            dirty();
        }
        seq = S_IDLE;
        msg = m;
    }

    // =================================================================================
    //  MANUAL GATES + SMART FLOWER DROP
    // =================================================================================
    private boolean flowPulse;       // manual BACK drop-gate pulse running
    private boolean flowerLatched;   // dpad_right: flower gate held open
    private boolean flowerPulse;     // smart single drop: gate open on a timer
    private boolean flowerPending;   // smart drop waiting for a snapshot first
    private double flowerMs;         // open time chosen for the current/last drop
    private int fDrop = E;           // type of the last dropped ball (telemetry)
    private int fNext = E;           // type of the ball that was behind it (telemetry)
    private final ElapsedTime flowT = new ElapsedTime();
    private final ElapsedTime flowerT = new ElapsedTime();

    private void gates() {
        // Timers first, so a pulse always closes on time, even while a sequence is running.
        if (flowPulse && flowT.milliseconds() >= FLOW_PULSE_MS) {
            out(FLOW, O_FLOW, stopFlow);
            flowPulse = false;
            dirty();   // an unknown number of balls left: re-index
        }
        if (flowerPulse && flowerT.milliseconds() >= flowerMs) {
            out(FLOWER, O_FLOWER, flowerClose);
            flowerPulse = false;
        }

        if (seq != S_IDLE) return;   // smart shoot owns the gates while it runs
        boolean busy = flowerPulse || flowerPending;

        if (pressed(B_Y)) fw(!fwBlocked);   // manual flywheel gate toggle (edge-detected, no sleep)

        if (pressed(B_BACK) && !flowPulse) {   // manual drop-gate pulse (old RT/Flow behaviour, non-blocking)
            out(FLOW, O_FLOW, letFlow);
            flowT.reset();
            flowPulse = true;
            snapValid = false;
        }

        if (pressed(B_DR) && !busy) {   // flower gate latch (old FlowerGate, edge-detected)
            flowerLatched = !flowerLatched;
            out(FLOWER, O_FLOWER, flowerLatched ? flowerOpen : flowerClose);
            dirty();   // balls may have left: re-index after it settles
        }

        if (pressed(B_DD) && !busy && !flowerLatched) {   // smart single drop
            if (snapValid) dropOne();   // order already known: drop immediately
            else {                      // order unknown: take a snapshot first, drop when it finishes
                snap();
                flowerPending = true;
            }
        }
        if (flowerPending && !snapping) {
            flowerPending = false;
            dropOne();
        }
    }

    */
/** Flower open time for the ball that drops. *//*

    private double flowerBase(int type) {
        if (type == P) return FLOWER_POLLEN_MS;
        if (type == N) return FLOWER_NECTAR_MS;
        return FLOWER_UNKNOWN_MS;
    }

    */
/** Adjustment for the ball BEHIND the dropping one (E = nothing behind, U = unknown: no change). *//*

    private double flowerTrim(int next) {
        if (next == P) return FLOWER_NEXT_POLLEN_TRIM_MS;
        if (next == N) return FLOWER_NEXT_NECTAR_TRIM_MS;
        if (next == E) return FLOWER_LAST_EXTRA_MS;
        return 0;
    }

    */
/**
     * Drop exactly one ball through the flower gate. Replaces the old FlowerOnce(), which
     * blocked the robot for 560 ms and always used the same time.
     *//*

    private void dropOne() {
        boolean far = FLOWER_FROM_FAR_END;
        boolean have = snapValid && qn > 0;   // do we actually know what's in the chamber?
        int di = far ? qn - 1 : 0;            // slot of the ball nearest the flower gate
        fDrop = have ? q[di] : U;             // no data: still drop (the driver asked), with the safe time
        fNext = !have ? U : qn > 1 ? q[far ? qn - 2 : 1] : E;
        if (have) {
            // Remove the dropped ball from q. The rest keep their order, so the next press
            // needs no new picture.
            for (int k = di; k < qn - 1; k++) q[k] = q[k + 1];
            q[--qn] = E;
        }
        flowerMs = Math.max(50, flowerBase(fDrop) + flowerTrim(fNext));   // 50 ms floor so a bad trim can't skip the drop
        out(FLOWER, O_FLOWER, flowerOpen);
        flowerT.reset();
        flowerPulse = true;   // gates() closes it when flowerMs has passed
    }

    // =================================================================================
    //  TELEMETRY (10 Hz)
    // =================================================================================
    private final StringBuilder qs = new StringBuilder(MAX);   // reused, so building text makes no garbage

    private void telem() {
        if (telT.milliseconds() < 100) return;   // building strings every 5 ms loop wastes CPU
        telT.reset();
        qs.setLength(0);
        for (int i = 0; i < MAX; i++) qs.append(CH.charAt(q[i]));

        // Seq: current step, plus which ball is moving (or the last result when idle).
        telemetry.addData("Seq", "%s %s", SN[seq],
                seq == S_FEED || seq == S_HOLD ? "ball " + (si + 1) + "/" + shotN : msg);
        // Chamber: the order the robot believes, gate first, and whether it can be trusted.
        telemetry.addData("Chamber gate>end", "[%s] %s %s", qs,
                snapping ? "indexing" : snapValid ? "valid" : "stale",
                cam == null ? "NO CAM" : camOk ? "cam ok" : "cam NO FRAMES");
        // Wheels: measured / target speed, ready flag, shots detected (should rise by 1 per ball).
        telemetry.addData("Nectar", "%.0f/%.0f %s fired %d", wv[N], on[N] ? NECTAR_VEL : 0,
                near(N) ? "RDY" : "--", shotCnt[N]);
        telemetry.addData("Pollen", "%.0f/%.0f %s fired %d", wv[P], on[P] ? POLLEN_VEL : 0,
                near(P) ? "RDY" : "--", shotCnt[P]);
        // Flower: gate state, last drop as dropped>behind, and the open time used.
        telemetry.addData("Flower", "%s %c>%c %.0fms",
                flowerLatched ? "OPEN" : flowerPulse ? "PULSE" : "closed",
                CH.charAt(fDrop), CH.charAt(fNext), flowerMs);
        // Drive direction and loop time. Expect roughly 5-10 ms; much higher means something is slow.
        telemetry.addData("Drive", "%s | loop %.1f ms", flip ? "FLIPPED" : "normal", loopMs);
        telemetry.update();
    }
}
*/
