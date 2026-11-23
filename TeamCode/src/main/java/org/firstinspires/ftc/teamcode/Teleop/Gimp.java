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

/*
 * АЛЬФА РУЧНОЙ ("Alpha manual") — BIOBUZZ TeleOp, two controllers, manual ball order.
 *
 * MECHANISM: slide chamber -> DROP GATE -> passive sorter -> Nectar / Pollen flywheel.
 * The drop gate is the only gate in the shooting path. It releases balls one at a time AND
 * pauses the next ball when that ball's flywheel still needs to recover from its last shot.
 * (The old flow gate under the sorter has been removed from the robot and from this code.)
 *
 * GAMEPAD 2 (operator): press each ball's colour as it enters the chamber.
 *   Y = pollen   B = red nectar   X = blue nectar   LB = undo last   BACK = clear all
 *
 * GAMEPAD 1 (driver):
 *   sticks = drive        RB = flip front          B / A = nectar / pollen wheel on
 *   X = wheels off + stop shooting                  LT / LB = intake in / out
 *   RT = SMART SHOOT      Y = drop gate open/close  dpad_up = slide up/down
 *   dpad_right = flower gate open/close             dpad_down = drop ONE ball into a flower
 *
 * Every "public static" value is live-editable in FTC Dashboard (192.168.43.1:8080/dash).
 * Dashboard changes are lost on restart: copy good values back into this file.
 */
@Config
@TeleOp(name = "АЛЬФА РУЧНОЙ")
public class LukePerfectTeleOp extends LinearOpMode {

    // Ball types. P and N double as flywheel indexes: W[P] = pollen wheel, W[N] = nectar wheel.
    static final int P = 0, N = 1;   // pollen, nectar (red and blue nectar are the same size -> same wheel)
    static final int U = 2, E = 3;   // unknown (nothing entered, so it needs BOTH wheels), empty
    static final int MAX = 4;        // the chamber holds up to 4 balls
    static final String CH = "PN?-"; // telemetry letter for each type

    // ============================== TUNABLES ==============================
    // Chamber layout
    public static boolean FIRST_IN_AT_GATE = true;     // first ball intaked rolls all the way to the drop gate
    public static boolean FLOWER_FROM_FAR_END = false; // flower gate is at the far end of the slide

    // Flywheels: encoder ticks/s. Must be reachable (~2800 max on a 6000 rpm motor).
    public static double NECTAR_VEL = 2600, POLLEN_VEL = 2600;
    public static double INTAKE_VEL = 7500;       // above max = full power, same as the old code
    public static double READY_TOL = 0.05;        // wheel within 5% of target = ready to shoot
    public static double DIP_FRAC = 0.08;         // ready wheel drops 8% = a ball was just fired
    public static boolean USE_VEL_READY = true;   // false = ignore wheel speed, use fixed pause times only

    // Servo positions (positional servos configured as CRServo, so power = position)
    public static double DROP_CLOSED = 0.35;      // old posUp: holds the balls in the chamber
    public static double DROP_OPEN = -0.95;       // old posDown: balls roll into the sorter. Swap if reversed.
    public static double flowerClose = 0, flowerOpen = 0.75;
    public static double unlockpos = 0.15;

    // Smart shoot timing (ms)
    public static double POLLEN_CLEAR_MS = 90;    // how long a rolling pollen takes to pass the drop gate
    public static double NECTAR_CLEAR_MS = 115;   // nectar is bigger, so the gate must stay open longer
    public static double FROM_REST_EXTRA_MS = 80; // extra time when the ball starts from a stop at the gate
    public static double PP_HOLD_MS = 350;        // longest pause the pollen wheel needs between its balls
    public static double NN_HOLD_MS = 450;        // longest pause the nectar wheel needs between its balls
    public static double GATE_MIN_MS = 100;       // gate stays shut at least this long (servo travel time)
    public static double SPIN_TIMEOUT_MS = 2000;  // first ball goes anyway if its wheel never reads ready
    public static double HOLD_MAX_MS = 1000;      // no later ball ever waits longer than this

    // Flower drop (ms): open time = base for the dropping ball + trim for the ball behind it
    public static double FLOWER_NECTAR_MS = 550, FLOWER_POLLEN_MS = 430, FLOWER_UNKNOWN_MS = 550;
    public static double FLOWER_NEXT_NECTAR_TRIM_MS = 0;   // big nectar behind can't slip through
    public static double FLOWER_NEXT_POLLEN_TRIM_MS = -40; // small pollen behind can: close sooner
    public static double FLOWER_LAST_EXTRA_MS = 100;       // nothing behind: hold open a bit longer

    // ============================== HARDWARE ==============================
    private final DcMotor[] D = new DcMotor[4];       // drive: LB, LF, RB, RF
    private final DcMotorEx[] W = new DcMotorEx[2];   // flywheels: [P] "POLLEN", [N] "NECTAR"
    private DcMotorEx INTAKE;
    private CRServo DROP, FLOWER;                     // drop gate (config "FLYWHEEL"), flower gate
    private List<LynxModule> hubs;
    private LinearSlideController slide;

    // Write cache: a hub message is only sent when a value changes (saves loop time on REV hubs).
    private static final int O_DROP = 4, O_FLOWER = 5, O_IN = 6, O_W = 7;   // slots 0-3 = drive; O_W+P / O_W+N
    private final double[] last = new double[9];

    private void out(DcMotorSimple d, int id, double v) { if (v != last[id]) { d.setPower(v); last[id] = v; } }
    private void vel(DcMotorEx m, int id, double v) { if (v != last[id]) { m.setVelocity(v); last[id] = v; } }

    // ============================== BUTTONS ==============================
    // Both gamepads packed into one int so pressed() gives a clean "just pressed" for any button.
    private static final int B_A = 1, B_B = 1 << 1, B_X = 1 << 2, B_Y = 1 << 3, B_RB = 1 << 4,
            B_DD = 1 << 5, B_DR = 1 << 6, B_RT = 1 << 7,
            G2_POLLEN = 1 << 8, G2_RED = 1 << 9, G2_BLUE = 1 << 10, G2_UNDO = 1 << 11, G2_CLEAR = 1 << 12;
    private int bNow, bPrev;

    private void readButtons(Gamepad g1, Gamepad g2) {
        bPrev = bNow;
        bNow = (g1.a ? B_A : 0) | (g1.b ? B_B : 0) | (g1.x ? B_X : 0) | (g1.y ? B_Y : 0)
                | (g1.right_bumper ? B_RB : 0) | (g1.dpad_down ? B_DD : 0) | (g1.dpad_right ? B_DR : 0)
                | (g1.right_trigger > 0.3 ? B_RT : 0)
                | (g2.y ? G2_POLLEN : 0) | (g2.b ? G2_RED : 0) | (g2.x ? G2_BLUE : 0)   // colour keys
                | (g2.left_bumper ? G2_UNDO : 0) | (g2.back ? G2_CLEAR : 0);
    }

    /** True only on the loop where the button goes down, so holding it does nothing extra. */
    private boolean pressed(int b) { return (bNow & b) != 0 && (bPrev & b) == 0; }

    private final ElapsedTime clock = new ElapsedTime(), loopT = new ElapsedTime(), telT = new ElapsedTime();
    private double loopMs;
    private boolean flip, dropOpen;
    private final double[] pw = new double[4];

    // ============================== OPMODE ==============================
    @Override
    public void runOpMode() {
        Arrays.fill(last, Double.NaN);   // forces the first command to every device to be sent
        Arrays.fill(q, '-');
        Arrays.fill(lastClr, -1e9);      // the first ball to each wheel never waits on spacing
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        String[] dn = {"LB", "LF", "RB", "RF"};
        for (int i = 0; i < 4; i++) {
            D[i] = hardwareMap.get(DcMotor.class, dn[i]);
            D[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            D[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        D[0].setDirection(DcMotor.Direction.REVERSE);
        D[1].setDirection(DcMotor.Direction.REVERSE);

        W[P] = hardwareMap.get(DcMotorEx.class, "POLLEN");
        W[N] = hardwareMap.get(DcMotorEx.class, "NECTAR");
        INTAKE = hardwareMap.get(DcMotorEx.class, "INTAKE");
        W[P].setDirection(DcMotor.Direction.REVERSE);
        INTAKE.setDirection(DcMotor.Direction.REVERSE);
        for (DcMotorEx m : new DcMotorEx[]{W[P], W[N], INTAKE}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);           // required for setVelocity
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast when turned off
        }

        DcMotor LS = hardwareMap.get(DcMotor.class, "LS");
        LS.setDirection(DcMotor.Direction.REVERSE);

        DROP = hardwareMap.get(CRServo.class, "FLYWHEEL");   // the drop gate's config name in the old code
        FLOWER = hardwareMap.get(CRServo.class, "FLOWER");
        FLOWER.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareMap.get(CRServo.class, "UNLOCK").setPower(unlockpos);
        drop(false);                         // start with the balls held in the chamber
        out(FLOWER, O_FLOWER, flowerClose);

        slide = new LinearSlideController(LS);   // slide must be fully DOWN at init
        slide.init();

        // Bulk reads: one hub message per loop returns every encoder/velocity. Turned on after
        // slide.init() so any encoder reads during init are always fresh.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        telemetry.addLine("Привет! GAMEPAD 2: Y pollen | B red nectar | X blue nectar | LB undo | BACK clear");
        telemetry.update();
        waitForStart();
        slide.start();

        while (opModeIsActive()) {
            loopMs = loopT.milliseconds();
            loopT.reset();
            for (LynxModule h : hubs) h.clearBulkCache();
            readButtons(gamepad1, gamepad2);

            operator();                       // gamepad 2 builds the ball list
            readWheels();                     // wheel buttons, speeds, shot detection
            drive(gamepad1);
            vel(INTAKE, O_IN, gamepad1.left_trigger > 0.1 ? INTAKE_VEL : gamepad1.left_bumper ? -INTAKE_VEL : 0);
            slide.update(gamepad1.dpad_up);
            sequence();                       // smart shoot
            gates();                          // manual drop gate + flower
            for (int w = 0; w < 2; w++) vel(W[w], O_W + w, on[w] ? tgt(w) : 0);
            telem();
        }
    }

    // ============================== BALL LIST (gamepad 2) ==============================
    // q holds colour letters in the order they were pressed: 'Y' pollen, 'R' red, 'B' blue nectar.
    private final char[] q = new char[MAX];
    private int qn;

    private void operator() {
        if (pressed(G2_POLLEN)) add('Y');
        if (pressed(G2_RED)) add('R');
        if (pressed(G2_BLUE)) add('B');
        if (pressed(G2_UNDO) && qn > 0) q[--qn] = '-';
        if (pressed(G2_CLEAR)) clearQ();
    }

    private void add(char c) {
        if (qn < MAX) { q[qn++] = c; gamepad2.rumble(100); }   // accepted: short buzz
        else gamepad2.rumble(600);                             // already 4: long buzz, ignored
    }

    private void clearQ() { Arrays.fill(q, '-'); qn = 0; }

    private static int type(char c) { return c == 'Y' ? P : N; }

    /** Index in q of the ball at position pos counted from the drop gate (0 = touching the gate). */
    private int qi(int pos) { return FIRST_IN_AT_GATE ? pos : qn - 1 - pos; }

    // ============================== DRIVE ==============================
    private void drive(Gamepad g) {
        if (pressed(B_RB)) flip = !flip;   // flipping negates forward and strafe, not turn
        double s = flip ? -1 : 1;
        double f = s * g.left_stick_y, x = -s * g.left_stick_x, r = g.right_stick_x;
        pw[0] = f - x + r;   // LB
        pw[1] = f + x + r;   // LF
        pw[2] = f + x - r;   // RB
        pw[3] = f - x - r;   // RF
        double m = Math.max(1, Math.max(Math.max(Math.abs(pw[0]), Math.abs(pw[1])),
                Math.max(Math.abs(pw[2]), Math.abs(pw[3]))));   // scale down together if any > 1
        for (int i = 0; i < 4; i++) out(D[i], i, pw[i] / m);
    }

    // ============================== FLYWHEELS ==============================
    private final boolean[] on = new boolean[2], armed = new boolean[2];
    private final double[] wv = new double[2];   // measured speed
    private final int[] shotCnt = new int[2];    // shots detected per wheel

    private double tgt(int w) { return w == N ? NECTAR_VEL : POLLEN_VEL; }

    /** Wheel w is on and within READY_TOL of its target speed. */
    private boolean near(int w) { return on[w] && Math.abs(wv[w] - tgt(w)) <= tgt(w) * READY_TOL; }

    private void readWheels() {
        if (pressed(B_B)) on[N] = true;
        if (pressed(B_A)) on[P] = true;
        if (pressed(B_X)) {
            on[P] = on[N] = false;
            if (seq != S_IDLE) finish("STOPPED (X) - re-enter balls on gamepad 2");
        }
        // Shot detection: a wheel at speed is "armed"; if it then dips, a ball went through.
        for (int w = 0; w < 2; w++) {
            wv[w] = W[w].getVelocity();
            if (!on[w]) armed[w] = false;
            else if (near(w)) armed[w] = true;
            else if (armed[w] && wv[w] < tgt(w) * (1 - DIP_FRAC)) { armed[w] = false; shotCnt[w]++; }
        }
    }

    // ============================== SMART SHOOT (RT) ==============================
    // WAIT: drop gate closed, the next ball waits for its flywheel (this is also the spin-up).
    // FEED: drop gate open, a ball is rolling through it.
    private static final int S_IDLE = 0, S_WAIT = 1, S_FEED = 2;
    private static final String[] SN = {"IDLE", "WAIT", "FEED"};
    private int seq = S_IDLE, si, shotN;             // step, current ball, balls this round
    private final int[] shots = new int[MAX];        // this round's order, gate first
    private final int[] relMark = new int[2];        // per wheel: shot count when its latest ball was released
    private final double[] lastClr = new double[2];  // per wheel: time its latest ball passed the gate
    private boolean fromRest;                        // current ball started from a stop
    private final ElapsedTime seqT = new ElapsedTime();
    private String msg = "ready";

    private void go(int s) { seq = s; seqT.reset(); }

    private void drop(boolean open) { dropOpen = open; out(DROP, O_DROP, open ? DROP_OPEN : DROP_CLOSED); }

    private void finish(String m) { drop(false); seq = S_IDLE; msg = m; }

    /** Release ball si, remembering each wheel's shot count so its dip can be matched to this ball. */
    private void feed(boolean rest) {
        fromRest = rest;
        for (int w = 0; w < 2; w++) if (shots[si] == U || shots[si] == w) relMark[w] = shotCnt[w];
        go(S_FEED);
    }

    /**
     * May ball si go now? Each wheel it needs must have recovered from ITS previous ball:
     *   (enough time has passed OR that ball was seen firing) AND the wheel is back at speed.
     * Two of the same type in a row are therefore always paused, and the pause ends as soon
     * as the wheel has really recovered. For the first ball this is simply "wheel at speed".
     */
    private boolean clearToFeed() {
        int t = shots[si];
        double now = clock.milliseconds();
        for (int w = 0; w < 2; w++) {
            if (t != U && t != w) continue;   // this ball doesn't use wheel w
            boolean spaced = now - lastClr[w] >= (w == P ? PP_HOLD_MS : NN_HOLD_MS);
            boolean fired = shotCnt[w] > relMark[w];
            if (USE_VEL_READY ? !((spaced || fired) && near(w)) : !spaced) return false;
        }
        return true;
    }

    private void sequence() {
        double t = seqT.milliseconds();
        if (seq == S_IDLE) {
            if (!pressed(B_RT) || flowerPulse) return;
            flowerLatched = false;
            out(FLOWER, O_FLOWER, flowerClose);   // never shoot with the flower gate open
            if (qn > 0) {                         // operator's list, gate first
                shotN = qn;
                for (int i = 0; i < qn; i++) shots[i] = type(q[qi(i)]);
                msg = "firing " + qn;
            } else {                              // nothing entered: assume 4 unknowns, full pauses
                shotN = MAX;
                Arrays.fill(shots, U);
                msg = "BLIND (nothing entered)";
            }
            clearQ();   // these balls are leaving; anything entered from now on is a new ball
            for (int i = 0; i < shotN; i++) {     // turn on only the wheels this round needs
                if (shots[i] != N) on[P] = true;
                if (shots[i] != P) on[N] = true;
            }
            si = 0;
            drop(false);
            go(S_WAIT);

        } else if (seq == S_WAIT) {
            // Open once the gate has been shut long enough AND the ball's wheel is ready,
            // or give up waiting (spin-up timeout for ball 1, hold limit after that).
            if ((t >= GATE_MIN_MS && clearToFeed()) || t >= (si == 0 ? SPIN_TIMEOUT_MS : HOLD_MAX_MS)) {
                drop(true);
                feed(true);   // the ball was sitting at the closed gate, so it starts from rest
            }

        } else {   // S_FEED
            double clear = (shots[si] == P ? POLLEN_CLEAR_MS : NECTAR_CLEAR_MS) + (fromRest ? FROM_REST_EXTRA_MS : 0);
            if (t < clear) return;   // ball still passing the gate
            for (int w = 0; w < 2; w++) if (shots[si] == U || shots[si] == w) lastClr[w] = clock.milliseconds();
            if (++si >= shotN) finish("done, fed " + shotN);   // last ball through: close up
            else if (clearToFeed()) feed(false);               // next wheel ready: gate stays open, ball rolls on
            else { drop(false); go(S_WAIT); }                  // next wheel busy: close the gate = the pause
        }
    }

    // ============================== MANUAL GATES + FLOWER ==============================
    private boolean flowerLatched, flowerPulse;
    private double flowerMs;
    private int fDrop = E, fNext = E;   // last flower drop, for telemetry
    private final ElapsedTime flowerT = new ElapsedTime();

    private void gates() {
        if (flowerPulse && flowerT.milliseconds() >= flowerMs) { out(FLOWER, O_FLOWER, flowerClose); flowerPulse = false; }
        if (seq != S_IDLE) return;   // smart shoot owns the drop gate while running

        if (pressed(B_Y)) drop(!dropOpen);   // manual drop gate (does not change the ball list)
        if (pressed(B_DR) && !flowerPulse) {
            flowerLatched = !flowerLatched;
            out(FLOWER, O_FLOWER, flowerLatched ? flowerOpen : flowerClose);
        }
        if (pressed(B_DD) && !flowerPulse && !flowerLatched) dropOne();
    }

    /** Drop one ball into a flower: open time depends on that ball and the one behind it. */
    private void dropOne() {
        int pos = FLOWER_FROM_FAR_END ? qn - 1 : 0;   // the ball nearest the flower gate
        fDrop = qn > 0 ? type(q[qi(pos)]) : U;        // list empty: drop anyway, safe timing
        fNext = qn > 1 ? type(q[qi(FLOWER_FROM_FAR_END ? qn - 2 : 1)]) : qn == 1 ? E : U;
        if (qn > 0) {                                 // remove the dropped ball from the list
            for (int k = qi(pos); k < qn - 1; k++) q[k] = q[k + 1];
            q[--qn] = '-';
        }
        double base = fDrop == P ? FLOWER_POLLEN_MS : fDrop == N ? FLOWER_NECTAR_MS : FLOWER_UNKNOWN_MS;
        double trim = fNext == P ? FLOWER_NEXT_POLLEN_TRIM_MS : fNext == N ? FLOWER_NEXT_NECTAR_TRIM_MS
                : fNext == E ? FLOWER_LAST_EXTRA_MS : 0;
        flowerMs = Math.max(50, base + trim);
        out(FLOWER, O_FLOWER, flowerOpen);
        flowerT.reset();
        flowerPulse = true;
    }

    // ============================== TELEMETRY (10 Hz) ==============================
    private final StringBuilder qs = new StringBuilder(MAX);

    private void telem() {
        if (telT.milliseconds() < 100) return;
        telT.reset();
        qs.setLength(0);
        for (int pos = 0; pos < MAX; pos++) qs.append(pos < qn ? q[qi(pos)] : '-');   // gate first
        telemetry.addData("Seq", "%s %s", SN[seq], seq == S_IDLE ? msg : "ball " + (si + 1) + "/" + shotN);
        telemetry.addData("Chamber gate>end", "[%s]  drop gate %s", qs, dropOpen ? "OPEN" : "closed");
        telemetry.addData("Nectar", "%.0f/%.0f %s fired %d", wv[N], on[N] ? NECTAR_VEL : 0, near(N) ? "RDY" : "--", shotCnt[N]);
        telemetry.addData("Pollen", "%.0f/%.0f %s fired %d", wv[P], on[P] ? POLLEN_VEL : 0, near(P) ? "RDY" : "--", shotCnt[P]);
        telemetry.addData("Flower", "%s %c>%c %.0fms", flowerLatched ? "OPEN" : flowerPulse ? "PULSE" : "closed",
                CH.charAt(fDrop), CH.charAt(fNext), flowerMs);
        telemetry.addData("Drive", "%s | loop %.1f ms", flip ? "FLIPPED" : "normal", loopMs);
        telemetry.update();
    }
}
