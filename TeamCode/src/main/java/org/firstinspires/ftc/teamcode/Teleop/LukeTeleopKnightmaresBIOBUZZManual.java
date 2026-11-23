package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

/**
 * =====================================================================================
 *  АЛЬФА РУЧНОЙ ("Alpha manual") — BIOBUZZ TeleOp, two controllers, MANUAL indexing
 * =====================================================================================
 *
 *  Same robot code as LukeTeleopKnightmaresBIOBUZZ.java (the camera version), except that
 *  the ball order comes from a second driver on GAMEPAD 2 instead of the Limelight. All
 *  camera code is removed. Use it for testing before the camera is set up, or as a backup
 *  if the camera fails.
 *
 *  GAMEPAD 2 (operator): press the button matching each ball's colour AS IT ENTERS the chamber.
 *    Y (yellow)   pollen
 *    B (red)      red nectar
 *    X (blue)     blue nectar
 *    LB           undo the last entry
 *    BACK         clear all entries
 *  Short rumble = entry accepted. Long rumble = rejected (chamber already holds 4).
 *  Colours match the Logitech F310 / Xbox layout. On a PS4-style controller the shapes
 *  differ (B is still red), so remap the three bits in readButtons() if needed.
 *
 *  GAMEPAD 1 (driver): identical to the camera version, minus dpad_left (no camera):
 *    sticks drive | RB flip front | B/A Nectar/Pollen wheel on | X wheels off + ABORT
 *    LT/LB intake | RT SMART SHOOT | Y flywheel gate | BACK drop-gate pulse
 *    dpad_up slide | dpad_right flower latch | dpad_down smart single flower drop
 *
 *  HOW THE ENTERED ORDER IS USED
 *    - RT: the entries are copied into the shooting order and the list is CLEARED at once.
 *      Those balls are leaving, so anything entered from then on is a new ball.
 *    - dpad_down: the ball at the flower gate is removed from the list automatically.
 *    - Nothing else changes the list (manual BACK pulse, flower latch, reversing the intake).
 *      If balls leave that way, fix the list with LB / BACK.
 *    - RT with nothing entered = BLIND mode: assumes 4 unknown balls, with the full wait
 *      between every ball. Slow but safe.
 *
 *  Shooting logic (spin-up, feed, per-wheel recovery holds, shot detection) is the same
 *  code as the camera version. Its comments explain each rule in more depth.
 */
@Config
@TeleOp(name = "АЛЬФА РУЧНОЙ")   // "Alpha manual": a different name, so both OpModes can be installed
public class LukeTeleopKnightmaresBIOBUZZManual extends LinearOpMode {

    // =================================================================================
    //  BALL TYPES
    // =================================================================================
    // P and N double as flywheel indexes (W[P] = pollen wheel, W[N] = nectar wheel).
    static final int P = 0;     // pollen -> pollen wheel
    static final int N = 1;     // nectar (red or blue: same size, same wheel) -> nectar wheel
    static final int U = 2;     // unknown (blind mode) -> needs BOTH wheels
    static final int E = 3;     // empty
    static final int MAX = 4;   // chamber capacity
    static final String CH = "PN?-";   // telemetry character per type

    // =================================================================================
    //  TUNABLES (FTC Dashboard: edits apply live but are lost on restart)
    // =================================================================================
    // Order of entry
    public static boolean FIRST_IN_AT_GATE = true;    // true: the first ball intaked rolls to the drop gate.
                                                      // false: it ends up at the FAR end (last in = at the gate).
    public static boolean FLOWER_FROM_FAR_END = false; // true if the flower gate is at the far end of the slide

    // Flywheels (ticks/s; must be reachable, see the camera version for why)
    public static double NECTAR_VEL = 2600;
    public static double POLLEN_VEL = 2600;
    public static double INTAKE_VEL = 7500;        // above top speed = full power (as the old code)
    public static double READY_TOL = 0.05;         // within 5% of target = ready
    public static double DIP_FRAC = 0.08;          // ready wheel dips 8% = one shot counted
    public static boolean USE_VEL_READY = true;    // false = fixed-time holds only
    public static boolean SPACE_ALL = true;        // false = only hold two SAME-type balls in a row
    public static boolean PRESTAGE = false;        // roll balls onto the closed flywheel gate during spin-up
    public static boolean SPIN_DOWN_AFTER = false; // turn wheels off after a smart shoot

    // Servo positions (positional servos configured as CRServo: setPower = position)
    public static double FW_PASS = 0.35;           // flywheel gate open   (old posUp)
    public static double FW_BLOCK = -0.95;         // flywheel gate closed (old posDown). Swap if reversed.
    public static double flowerClose = 0;
    public static double flowerOpen = 0.75;
    public static double stopFlow = -0.4;          // drop gate closed
    public static double letFlow = 0;              // drop gate open
    public static double unlockpos = 0.15;

    // Smart shoot timing (ms)
    public static double SPIN_MIN_MS = 150;        // minimum spin-up so speed readings settle
    public static double SPIN_TIMEOUT_MS = 2000;   // shoot anyway if wheels never read ready
    public static double FIRST_ARRIVAL_MS = 250;   // drop gate opens -> ball 0 reaches the flywheel gate
    public static double POLLEN_CLEAR_MS = 90;     // rolling pollen fully past the flywheel gate
    public static double NECTAR_CLEAR_MS = 115;    // rolling nectar (bigger) fully past the gate
    public static double FROM_REST_EXTRA_MS = 80;  // extra time for a ball starting from a stop
    public static double PP_HOLD_MS = 350;         // pollen wheel recovery cap
    public static double NN_HOLD_MS = 450;         // nectar wheel recovery cap
    public static double GATE_MIN_MS = 60;         // min time the flywheel gate stays shut
    public static double HOLD_MAX_MS = 1000;       // absolute max hold per ball
    public static double FINAL_FIRE_MS = 400;      // last ball through its wheel before closing up
    public static double FLOW_PULSE_MS = 400;      // manual BACK drop-gate pulse

    // Smart flower drop (ms): base for the dropping ball + trim for the ball behind it
    public static double FLOWER_NECTAR_MS = 550;
    public static double FLOWER_POLLEN_MS = 430;
    public static double FLOWER_UNKNOWN_MS = 550;          // list empty: safe (nectar) time
    public static double FLOWER_NEXT_NECTAR_TRIM_MS = 0;
    public static double FLOWER_NEXT_POLLEN_TRIM_MS = -40; // small pollen behind: close earlier
    public static double FLOWER_LAST_EXTRA_MS = 100;       // nothing behind: no rush

    // =================================================================================
    //  HARDWARE (same config names and ports as before; no Limelight needed)
    // =================================================================================
    private final DcMotor[] D = new DcMotor[4];       // LB 0C, LF 1C, RB 2C, RF 3C
    private final DcMotorEx[] W = new DcMotorEx[2];   // [P] "POLLEN" 1E, [N] "NECTAR" 0E
    private DcMotorEx INTAKE;                         // "INTAKE" 3E
    private CRServo FLOW;                             // "FLOW" 1S     drop gate
    private CRServo FLOWER;                           // "FLOWER" 2S   flower gate
    private CRServo FWGATE;                           // "FLYWHEEL" 5S flywheel gate
    private List<LynxModule> hubs;                    // for bulk reads
    private LinearSlideController slide;              // drives "LS" 2E

    // Write cache: only message a hub when a value changes (slots 0-3 = drive motors).
    private static final int O_FW = 4, O_FLOWER = 5, O_FLOW = 6, O_IN = 7, O_W = 8;   // O_W + P / N
    private final double[] last = new double[10];

    private void out(DcMotorSimple d, int id, double v) { if (v != last[id]) { d.setPower(v); last[id] = v; } }
    private void vel(DcMotorEx m, int id, double v) { if (v != last[id]) { m.setVelocity(v); last[id] = v; } }

    // =================================================================================
    //  BUTTONS: both gamepads packed into one int, so pressed() works for either.
    // =================================================================================
    // Gamepad 1
    private static final int B_A = 1, B_B = 1 << 1, B_X = 1 << 2, B_Y = 1 << 3, B_RB = 1 << 4,
            B_DD = 1 << 5, B_DR = 1 << 6, B_BACK = 1 << 7, B_RT = 1 << 8;
    // Gamepad 2 (operator)
    private static final int G2_POLLEN = 1 << 9, G2_RED = 1 << 10, G2_BLUE = 1 << 11,
            G2_UNDO = 1 << 12, G2_CLEAR = 1 << 13;
    private int bNow, bPrev;

    private void readButtons(Gamepad g1, Gamepad g2) {
        bPrev = bNow;
        bNow = (g1.a ? B_A : 0) | (g1.b ? B_B : 0) | (g1.x ? B_X : 0) | (g1.y ? B_Y : 0)
                | (g1.right_bumper ? B_RB : 0) | (g1.dpad_down ? B_DD : 0) | (g1.dpad_right ? B_DR : 0)
                | (g1.back ? B_BACK : 0) | (g1.right_trigger > 0.3 ? B_RT : 0)
                // Colour keys: change these three if your controller's colours differ.
                | (g2.y ? G2_POLLEN : 0)   // yellow button = yellow pollen
                | (g2.b ? G2_RED : 0)      // red button    = red nectar
                | (g2.x ? G2_BLUE : 0)     // blue button   = blue nectar
                | (g2.left_bumper ? G2_UNDO : 0) | (g2.back ? G2_CLEAR : 0);
    }

    /** True only on the loop where the button goes from up to down (one press = one action). */
    private boolean pressed(int b) { return (bNow & b) != 0 && (bPrev & b) == 0; }

    // ---- Loop state ----
    private final ElapsedTime clock = new ElapsedTime();   // free-running ms clock
    private final ElapsedTime loopT = new ElapsedTime();
    private final ElapsedTime telT = new ElapsedTime();
    private double loopMs;
    private boolean flip, fwBlocked;
    private final double[] pw = new double[4];

    // =================================================================================
    //  OPMODE
    // =================================================================================
    @Override
    public void runOpMode() {
        Arrays.fill(last, Double.NaN);   // first write to every device always goes out
        Arrays.fill(q, '-');
        Arrays.fill(lastClr, -1e9);      // first ball to each wheel never waits
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        String[] dn = {"LB", "LF", "RB", "RF"};
        for (int i = 0; i < 4; i++) {
            D[i] = hardwareMap.get(DcMotor.class, dn[i]);
            D[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            D[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        D[0].setDirection(DcMotor.Direction.REVERSE);   // LB
        D[1].setDirection(DcMotor.Direction.REVERSE);   // LF

        W[P] = hardwareMap.get(DcMotorEx.class, "POLLEN");
        W[N] = hardwareMap.get(DcMotorEx.class, "NECTAR");
        INTAKE = hardwareMap.get(DcMotorEx.class, "INTAKE");
        W[P].setDirection(DcMotor.Direction.REVERSE);
        INTAKE.setDirection(DcMotor.Direction.REVERSE);
        for (DcMotorEx m : new DcMotorEx[]{W[P], W[N], INTAKE}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);           // needed for setVelocity
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast when off
        }

        DcMotor LS = hardwareMap.get(DcMotor.class, "LS");
        LS.setDirection(DcMotor.Direction.REVERSE);

        FLOW = hardwareMap.get(CRServo.class, "FLOW");
        FLOWER = hardwareMap.get(CRServo.class, "FLOWER");
        FWGATE = hardwareMap.get(CRServo.class, "FLYWHEEL");
        FLOWER.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareMap.get(CRServo.class, "UNLOCK").setPower(unlockpos);
        fw(false);
        out(FLOWER, O_FLOWER, flowerClose);
        out(FLOW, O_FLOW, stopFlow);

        slide = new LinearSlideController(LS);   // slide must be fully down at init
        slide.init();

        // Bulk reads, enabled after slide.init() (see the camera version for why).
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        telemetry.addLine("Привет! Manual index on GAMEPAD 2:");
        telemetry.addLine("Y pollen | B red nectar | X blue nectar | LB undo | BACK clear");
        telemetry.update();

        waitForStart();
        slide.start();

        while (opModeIsActive()) {
            loopMs = loopT.milliseconds();
            loopT.reset();
            for (LynxModule h : hubs) h.clearBulkCache();
            readButtons(gamepad1, gamepad2);

            operator();              // gamepad 2: build the ball list
            readWheels();            // wheel speeds, A/B/X, shot detection
            drive(gamepad1);
            intake(gamepad1);
            slide.update(gamepad1.dpad_up);
            sequence();              // smart shoot (RT)
            gates();                 // manual gates + smart flower drop
            for (int w = 0; w < 2; w++) vel(W[w], O_W + w, on[w] ? tgt(w) : 0);
            telem();
        }
    }

    // =================================================================================
    //  OPERATOR BALL LIST (gamepad 2)
    // =================================================================================
    // q holds entries in the order they were PRESSED: 'Y' pollen, 'R' red nectar, 'B' blue nectar.
    // Storing the colour letter (not just pollen/nectar) lets telemetry show exactly what was
    // entered, so the operator can check it against the chamber. type() converts for shooting.
    private final char[] q = new char[MAX];
    private int qn;   // number of entries

    private void operator() {
        if (pressed(G2_POLLEN)) add('Y');
        if (pressed(G2_RED)) add('R');
        if (pressed(G2_BLUE)) add('B');
        if (pressed(G2_UNDO) && qn > 0) q[--qn] = '-';   // remove the most recent entry
        if (pressed(G2_CLEAR)) clearQ();
    }

    private void add(char c) {
        if (qn < MAX) { q[qn++] = c; gamepad2.rumble(100); }   // accepted: short buzz
        else gamepad2.rumble(600);                             // already 4: long buzz, ignored
    }   // (Logitech F310s have no rumble motor; the call is harmless.)

    private void clearQ() { Arrays.fill(q, '-'); qn = 0; }

    /** Colour letter -> ball type. Red and blue nectar are the same size, so they share a wheel. */
    private static int type(char c) { return c == 'Y' ? P : N; }

    /**
     * Index in q of the ball at position pos counted FROM THE DROP GATE (0 = touching the gate).
     * Entries are in intake order; FIRST_IN_AT_GATE says which end of the chamber that is.
     */
    private int qi(int pos) { return FIRST_IN_AT_GATE ? pos : qn - 1 - pos; }

    // =================================================================================
    //  DRIVE / INTAKE (unchanged from the camera version)
    // =================================================================================
    private void drive(Gamepad g) {
        if (pressed(B_RB)) flip = !flip;
        double s = flip ? -1 : 1;
        double f = s * g.left_stick_y, x = -s * g.left_stick_x, r = g.right_stick_x;
        pw[0] = f - x + r;   // LB
        pw[1] = f + x + r;   // LF
        pw[2] = f + x - r;   // RB
        pw[3] = f - x - r;   // RF
        double m = Math.max(1, Math.max(Math.max(Math.abs(pw[0]), Math.abs(pw[1])),
                Math.max(Math.abs(pw[2]), Math.abs(pw[3]))));
        for (int i = 0; i < 4; i++) out(D[i], i, pw[i] / m);
    }

    private void intake(Gamepad g) {
        // No camera to re-index, so the intake no longer touches the ball list.
        vel(INTAKE, O_IN, g.left_trigger > 0.1 ? INTAKE_VEL : g.left_bumper ? -INTAKE_VEL : 0);
    }

    // =================================================================================
    //  FLYWHEELS + SHOT DETECTION (unchanged)
    // =================================================================================
    private final boolean[] on = new boolean[2], armed = new boolean[2];
    private final double[] wv = new double[2];
    private final int[] shotCnt = new int[2];

    private double tgt(int w) { return w == N ? NECTAR_VEL : POLLEN_VEL; }

    private boolean near(int w) {
        double t = tgt(w);
        return on[w] && Math.abs(wv[w] - t) <= t * READY_TOL;
    }

    private boolean ready(int type) {
        if (!USE_VEL_READY) return true;
        return type == U ? near(P) && near(N) : near(type);
    }

    private void readWheels() {
        if (pressed(B_B)) on[N] = true;
        if (pressed(B_A)) on[P] = true;
        if (pressed(B_X)) {
            on[P] = on[N] = false;
            if (seq != S_IDLE) end("ABORTED (X) - re-enter balls on gamepad 2", true);
        }
        for (int w = 0; w < 2; w++) {
            wv[w] = W[w].getVelocity();
            // at speed -> armed; armed and dipped -> one shot counted
            if (!on[w]) armed[w] = false;
            else if (near(w)) armed[w] = true;
            else if (armed[w] && wv[w] < tgt(w) * (1 - DIP_FRAC)) { armed[w] = false; shotCnt[w]++; }
        }
    }

    // =================================================================================
    //  SMART SHOOT (RT):  IDLE -> SPIN-UP -> FEED <-> HOLD -> FINISH
    // =================================================================================
    private static final int S_IDLE = 0, S_SPIN = 1, S_FEED = 2, S_HOLD = 3, S_DONE = 4;
    private static final String[] SN = {"IDLE", "SPIN-UP", "FEED", "HOLD", "FINISH"};
    private int seq = S_IDLE, si, shotN;
    private final int[] shots = new int[MAX];        // this sequence's order, gate first
    private final int[] relMark = new int[2];        // per wheel: shot count when its latest ball was released
    private final double[] lastClr = new double[2];  // per wheel: time its latest ball passed the gate
    private boolean fromRest;
    private final ElapsedTime seqT = new ElapsedTime();
    private String msg = "ready";

    private void go(int s) { seq = s; seqT.reset(); }

    private void fw(boolean block) { fwBlocked = block; out(FWGATE, O_FW, block ? FW_BLOCK : FW_PASS); }

    private double clearTime(int type) { return type == P ? POLLEN_CLEAR_MS : NECTAR_CLEAR_MS; }

    private double holdCap(int w) { return w == P ? PP_HOLD_MS : NN_HOLD_MS; }

    /** Release ball si; note each wheel's shot count so its dip can be matched to this ball. */
    private void feed(boolean rest) {
        fromRest = rest;
        for (int w = 0; w < 2; w++) if (shots[si] == U || shots[si] == w) relMark[w] = shotCnt[w];
        go(S_FEED);
    }

    /**
     * May ball si go in now? Its wheel(s) must have recovered from that wheel's previous ball:
     * (spaced by the cap OR a shot was detected) AND back at speed. Two of the same type in a
     * row are always held; the hold ends as soon as the wheel has fired and recovered.
     */
    private boolean clearToFeed() {
        int t = shots[si];
        if (!SPACE_ALL && shots[si - 1] != t && t != U && shots[si - 1] != U) return ready(t);
        double now = clock.milliseconds();
        for (int w = 0; w < 2; w++) {
            if (t != U && t != w) continue;
            boolean spaced = now - lastClr[w] >= holdCap(w);
            boolean shot = shotCnt[w] > relMark[w];
            if (USE_VEL_READY ? !((spaced || shot) && near(w)) : !spaced) return false;
        }
        return true;
    }

    private void sequence() {
        double t = seqT.milliseconds();
        switch (seq) {

            case S_IDLE:
                if (!pressed(B_RT) || flowerPulse || flowPulse) return;
                flowerLatched = false;
                out(FLOWER, O_FLOWER, flowerClose);   // never shoot through an open flower gate
                // Take the operator's list, gate first. Nothing entered -> blind mode.
                if (qn > 0) {
                    shotN = qn;
                    for (int i = 0; i < qn; i++) shots[i] = type(q[qi(i)]);
                    msg = "firing " + qn;
                } else {
                    shotN = MAX;
                    Arrays.fill(shots, U);
                    msg = "BLIND (nothing entered)";
                }
                clearQ();   // these balls are leaving; new entries from now on are new balls
                // Turn on only the wheels these balls need (plus any already on).
                for (int i = 0; i < shotN; i++) {
                    if (shots[i] != N) on[P] = true;
                    if (shots[i] != P) on[N] = true;
                }
                if (PRESTAGE) { fw(true); out(FLOW, O_FLOW, letFlow); }
                go(S_SPIN);
                return;

            case S_SPIN: {
                boolean ok = t >= SPIN_MIN_MS && (!PRESTAGE || t >= FIRST_ARRIVAL_MS);
                for (int i = 0; i < shotN; i++) ok &= ready(shots[i]);
                if (!ok && t < SPIN_TIMEOUT_MS) return;
                out(FLOW, O_FLOW, letFlow);   // drop gate open: balls roll
                fw(false);
                si = 0;
                feed(PRESTAGE);
                return;
            }

            case S_FEED: {
                double win = clearTime(shots[si])
                        + (si == 0 && !PRESTAGE ? FIRST_ARRIVAL_MS : 0)
                        + (fromRest ? FROM_REST_EXTRA_MS : 0);
                if (t < win) return;   // ball si still passing the gate
                for (int w = 0; w < 2; w++) if (shots[si] == U || shots[si] == w) lastClr[w] = clock.milliseconds();
                if (++si >= shotN) { go(S_DONE); return; }
                if (clearToFeed()) { feed(false); return; }   // wheel ready: next ball rolls straight in
                fw(true);                                     // wheel busy: block the next ball
                go(S_HOLD);
                return;
            }

            case S_HOLD:
                if ((t >= GATE_MIN_MS && clearToFeed()) || t >= HOLD_MAX_MS) { fw(false); feed(true); }
                return;

            case S_DONE:
                if (t >= FINAL_FIRE_MS) end("done, fed " + shotN, SPIN_DOWN_AFTER);
                return;
        }
    }

    /** Close up after a finished or aborted sequence. The ball list was already cleared at RT. */
    private void end(String m, boolean stopWheels) {
        out(FLOW, O_FLOW, stopFlow);
        fw(false);
        if (stopWheels) on[P] = on[N] = false;
        seq = S_IDLE;
        msg = m;
    }

    // =================================================================================
    //  MANUAL GATES + SMART FLOWER DROP
    // =================================================================================
    private boolean flowPulse, flowerLatched, flowerPulse;
    private double flowerMs;
    private int fDrop = E, fNext = E;   // last drop, for telemetry
    private final ElapsedTime flowT = new ElapsedTime(), flowerT = new ElapsedTime();

    private void gates() {
        if (flowPulse && flowT.milliseconds() >= FLOW_PULSE_MS) { out(FLOW, O_FLOW, stopFlow); flowPulse = false; }
        if (flowerPulse && flowerT.milliseconds() >= flowerMs) { out(FLOWER, O_FLOWER, flowerClose); flowerPulse = false; }
        if (seq != S_IDLE) return;   // smart shoot owns the gates while running

        if (pressed(B_Y)) fw(!fwBlocked);
        if (pressed(B_BACK) && !flowPulse) { out(FLOW, O_FLOW, letFlow); flowT.reset(); flowPulse = true; }
        if (pressed(B_DR) && !flowerPulse) {
            flowerLatched = !flowerLatched;
            out(FLOWER, O_FLOWER, flowerLatched ? flowerOpen : flowerClose);
        }
        if (pressed(B_DD) && !flowerPulse && !flowerLatched) dropOne();
    }

    private double flowerBase(int type) {
        return type == P ? FLOWER_POLLEN_MS : type == N ? FLOWER_NECTAR_MS : FLOWER_UNKNOWN_MS;
    }

    private double flowerTrim(int next) {
        if (next == P) return FLOWER_NEXT_POLLEN_TRIM_MS;
        if (next == N) return FLOWER_NEXT_NECTAR_TRIM_MS;
        return next == E ? FLOWER_LAST_EXTRA_MS : 0;   // unknown: no trim
    }

    /** Drop one ball through the flower gate, timed from the operator's list, then remove it. */
    private void dropOne() {
        int pos = FLOWER_FROM_FAR_END ? qn - 1 : 0;    // position (from the gate) of the ball at the flower gate
        int nextPos = FLOWER_FROM_FAR_END ? qn - 2 : 1;
        fDrop = qn > 0 ? type(q[qi(pos)]) : U;         // list empty: drop anyway with the safe time
        fNext = qn > 1 ? type(q[qi(nextPos)]) : qn == 1 ? E : U;
        if (qn > 0) {                                  // remove the dropped ball from the list
            for (int k = qi(pos); k < qn - 1; k++) q[k] = q[k + 1];
            q[--qn] = '-';
        }
        flowerMs = Math.max(50, flowerBase(fDrop) + flowerTrim(fNext));
        out(FLOWER, O_FLOWER, flowerOpen);
        flowerT.reset();
        flowerPulse = true;
    }

    // =================================================================================
    //  TELEMETRY (10 Hz)
    // =================================================================================
    private final StringBuilder qs = new StringBuilder(MAX);

    private void telem() {
        if (telT.milliseconds() < 100) return;
        telT.reset();
        qs.setLength(0);
        for (int pos = 0; pos < MAX; pos++) qs.append(pos < qn ? q[qi(pos)] : '-');   // gate first
        telemetry.addData("Seq", "%s %s", SN[seq],
                seq == S_FEED || seq == S_HOLD ? "ball " + (si + 1) + "/" + shotN : msg);
        telemetry.addData("Chamber gate>end", "[%s]  GP2: Y pollen  B red  X blue  LB undo  BACK clear", qs);
        telemetry.addData("Nectar", "%.0f/%.0f %s fired %d", wv[N], on[N] ? NECTAR_VEL : 0,
                near(N) ? "RDY" : "--", shotCnt[N]);
        telemetry.addData("Pollen", "%.0f/%.0f %s fired %d", wv[P], on[P] ? POLLEN_VEL : 0,
                near(P) ? "RDY" : "--", shotCnt[P]);
        telemetry.addData("Flower", "%s %c>%c %.0fms",
                flowerLatched ? "OPEN" : flowerPulse ? "PULSE" : "closed",
                CH.charAt(fDrop), CH.charAt(fNext), flowerMs);
        telemetry.addData("Drive", "%s | loop %.1f ms", flip ? "FLIPPED" : "normal", loopMs);
        telemetry.update();
    }
}
