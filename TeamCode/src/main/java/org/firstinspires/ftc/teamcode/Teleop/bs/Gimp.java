/*
package org.firstinspires.ftc.teamcode.Teleop.bs;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Teleop.V1.LinearSlideController;

import java.util.Arrays;
import java.util.List;

*/
/*
 * GIMP TeleOp — BIOBUZZ TeleOp, two controllers, manual ball order.
 *
 * MECHANISM
 *   slide chamber -> DROP GATE -> passive sorter -> Nectar flywheel / Pollen flywheel
 *   The drop gate is the only gate in the shooting path. It releases balls one at a time
 *   AND closes to pause the next ball while that ball's flywheel recovers from its last shot.
 *
 * GAMEPAD 2 (operator): press each ball's colour as it enters the chamber.
 *   Y = pollen    B = red nectar    X = blue nectar    LB = undo last    BACK = clear all
 *
 * GAMEPAD 1 (driver):
 *   sticks = drive                 RB = flip which end is the front
 *   B / A = nectar / pollen wheel on    X = wheels off and stop shooting
 *   LT / LB = intake in / out      RT = SMART SHOOT
 *   Y = drop gate open / close     dpad_up = slide up / down
 *   dpad_right = flower gate open / close    dpad_down = drop ONE ball into a flower
 *
 * TUNING
 *   Every "public static" value can be changed live in FTC Dashboard
 *   (192.168.43.1:8080/dash). Dashboard changes are lost on restart, so copy good
 *   values back into this file.
 *//*

@Config
@TeleOp(name = "GIMP TeleOp")
public class Gimp extends LinearOpMode {

    // =====================================================================
    //  BALL TYPES
    // =====================================================================
    // POLLEN and NECTAR are 0 and 1 so they double as flywheel numbers:
    // flywheels[POLLEN] is the pollen wheel, flywheels[NECTAR] is the nectar wheel.
    static final int POLLEN = 0;
    static final int NECTAR = 1;    // red and blue nectar are the same size, so they share a wheel
    static final int UNKNOWN = 2;   // used when nothing was entered: needs BOTH wheels ready
    static final int EMPTY = 3;
    static final int CHAMBER_SIZE = 4;
    static final String TYPE_LETTERS = "PN?-";   // telemetry letter for POLLEN, NECTAR, UNKNOWN, EMPTY

    // =====================================================================
    //  TUNABLES (live in FTC Dashboard)
    // =====================================================================

    // ---- Chamber layout ----
    public static boolean FIRST_IN_AT_GATE = true;       // first ball intaked rolls all the way to the drop gate
    public static boolean FLOWER_FROM_FAR_END = false;   // true if the flower gate is at the far end of the slide

    // ---- Flywheels (encoder ticks per second; must be reachable, ~2800 max on a 6000 rpm motor) ----
    public static double NECTAR_VEL = 2600;
    public static double POLLEN_VEL = 2600;
    public static double INTAKE_VEL = 7500;        // above max speed = full power, same as the old code
    public static double READY_TOL = 0.05;         // wheel within 5% of target = ready to shoot
    public static double DIP_FRAC = 0.08;          // ready wheel drops 8% = a ball was just fired
    public static boolean USE_VEL_READY = true;    // false = ignore wheel speed, use fixed pause times only

    // ---- Servo positions (positional servos configured as CRServo, so power = position) ----
    public static double DROP_CLOSED = 0.35;       // old posUp: holds the balls in the chamber
    public static double DROP_OPEN = -0.95;        // old posDown: balls roll into the sorter. Swap if reversed.
    public static double flowerClose = 0;
    public static double flowerOpen = 0.75;
    public static double unlockpos = 0.15;

    // ---- Smart shoot timing (milliseconds) ----
    public static double POLLEN_CLEAR_MS = 90;     // time for a rolling pollen to pass the drop gate
    public static double NECTAR_CLEAR_MS = 115;    // nectar is bigger, so the gate stays open longer
    public static double FROM_REST_EXTRA_MS = 80;  // extra time when the ball starts from a stop at the gate
    public static double PP_HOLD_MS = 350;         // longest pause the pollen wheel needs between its balls
    public static double NN_HOLD_MS = 450;         // longest pause the nectar wheel needs between its balls
    public static double GATE_MIN_MS = 100;        // gate stays shut at least this long (servo travel time)
    public static double SPIN_TIMEOUT_MS = 2000;   // first ball goes anyway if its wheel never reads ready
    public static double HOLD_MAX_MS = 1000;       // no later ball ever waits longer than this

    // ---- Flower drop timing (milliseconds): open time = base for the dropping ball + trim for the ball behind ----
    public static double FLOWER_NECTAR_MS = 550;
    public static double FLOWER_POLLEN_MS = 430;
    public static double FLOWER_UNKNOWN_MS = 550;           // list empty: use the safe (nectar) time
    public static double FLOWER_NEXT_NECTAR_TRIM_MS = 0;    // big nectar behind can't slip through
    public static double FLOWER_NEXT_POLLEN_TRIM_MS = -40;  // small pollen behind can, so close sooner
    public static double FLOWER_LAST_EXTRA_MS = 100;        // nothing behind: hold open a bit longer

    // =====================================================================
    //  HARDWARE
    // =====================================================================
    private final DcMotor[] driveMotors = new DcMotor[4];      // LB, LF, RB, RF
    private final DcMotorEx[] flywheels = new DcMotorEx[2];    // [POLLEN] "POLLEN", [NECTAR] "NECTAR"
    private DcMotorEx intake;
    private CRServo dropGate;                                  // config name "FLYWHEEL"
    private CRServo flowerGate;                                // config name "FLOWER"
    private List<LynxModule> hubs;
    private LinearSlideController slide;

    // =====================================================================
    //  WRITE CACHE
    // =====================================================================
    // Every command is a message to a hub, and those messages are the slowest part of a loop.
    // Each device gets a slot here; a command is only sent when its value actually changes.
    private static final int SLOT_DROP_GATE = 4;     // slots 0-3 are the four drive motors
    private static final int SLOT_FLOWER_GATE = 5;
    private static final int SLOT_INTAKE = 6;
    private static final int SLOT_FLYWHEEL = 7;      // 7 = pollen wheel, 8 = nectar wheel
    private final double[] lastSent = new double[9];

    private void setPowerIfChanged(DcMotorSimple device, int slot, double power) {
        if (power != lastSent[slot]) {
            device.setPower(power);
            lastSent[slot] = power;
        }
    }

    private void setVelocityIfChanged(DcMotorEx motor, int slot, double ticksPerSecond) {
        if (ticksPerSecond != lastSent[slot]) {
            motor.setVelocity(ticksPerSecond);
            lastSent[slot] = ticksPerSecond;
        }
    }

    // =====================================================================
    //  BUTTONS
    // =====================================================================
    // Each button is one bit. Comparing this loop's bits with last loop's bits tells us
    // which buttons were JUST pressed, so holding a button only triggers its action once.

    // Gamepad 1 (driver)
    private static final int BTN_A = 1;
    private static final int BTN_B = 1 << 1;
    private static final int BTN_X = 1 << 2;
    private static final int BTN_Y = 1 << 3;
    private static final int BTN_RIGHT_BUMPER = 1 << 4;
    private static final int BTN_DPAD_DOWN = 1 << 5;
    private static final int BTN_DPAD_RIGHT = 1 << 6;
    private static final int BTN_RIGHT_TRIGGER = 1 << 7;

    // Gamepad 2 (operator)
    private static final int OP_POLLEN = 1 << 8;
    private static final int OP_RED_NECTAR = 1 << 9;
    private static final int OP_BLUE_NECTAR = 1 << 10;
    private static final int OP_UNDO = 1 << 11;
    private static final int OP_CLEAR = 1 << 12;

    private int buttonsNow;
    private int buttonsBefore;

    private void readButtons() {
        buttonsBefore = buttonsNow;
        int b = 0;

        if (gamepad1.a) b |= BTN_A;
        if (gamepad1.b) b |= BTN_B;
        if (gamepad1.x) b |= BTN_X;
        if (gamepad1.y) b |= BTN_Y;
        if (gamepad1.right_bumper) b |= BTN_RIGHT_BUMPER;
        if (gamepad1.dpad_down) b |= BTN_DPAD_DOWN;
        if (gamepad1.dpad_right) b |= BTN_DPAD_RIGHT;
        if (gamepad1.right_trigger > 0.3) b |= BTN_RIGHT_TRIGGER;

        if (gamepad2.y) b |= OP_POLLEN;        // yellow button = yellow pollen
        if (gamepad2.b) b |= OP_RED_NECTAR;    // red button    = red nectar
        if (gamepad2.x) b |= OP_BLUE_NECTAR;   // blue button   = blue nectar
        if (gamepad2.left_bumper) b |= OP_UNDO;
        if (gamepad2.back) b |= OP_CLEAR;

        buttonsNow = b;
    }

    */
/** True only on the one loop where the button goes from up to down. *//*

    private boolean justPressed(int button) {
        boolean downNow = (buttonsNow & button) != 0;
        boolean downBefore = (buttonsBefore & button) != 0;
        return downNow && !downBefore;
    }

    // =====================================================================
    //  OPMODE
    // =====================================================================
    private final ElapsedTime clock = new ElapsedTime();       // free-running clock for timestamps
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private double loopMs;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Привет! GAMEPAD 2: Y pollen | B red nectar | X blue nectar | LB undo | BACK clear");
        telemetry.update();
        waitForStart();
        slide.start();

        while (opModeIsActive()) {
            loopMs = loopTimer.milliseconds();
            loopTimer.reset();
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();          // fresh sensor data for this loop
            }

            readButtons();
            readOperator();                    // gamepad 2 builds the ball list
            updateFlywheels();                 // wheel buttons, measured speeds, shot detection
            updateDrive();
            updateIntake();
            slide.update(gamepad1.dpad_up);
            updateShooting();                  // smart shoot (RT)
            updateGates();                     // manual drop gate + flower gate
            sendFlywheelSpeeds();
            updateTelemetry();
        }
    }

    private void initHardware() {
        Arrays.fill(lastSent, Double.NaN);     // forces the first command to every device to be sent
        Arrays.fill(ballList, '-');
        Arrays.fill(lastPassTime, -1e9);       // the first ball to each wheel never waits on spacing

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        // Drive motors
        String[] driveNames = {"LB", "LF", "RB", "RF"};
        for (int i = 0; i < 4; i++) {
            driveMotors[i] = hardwareMap.get(DcMotor.class, driveNames[i]);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        driveMotors[0].setDirection(DcMotor.Direction.REVERSE);   // LB
        driveMotors[1].setDirection(DcMotor.Direction.REVERSE);   // LF

        // Flywheels and intake
        flywheels[POLLEN] = hardwareMap.get(DcMotorEx.class, "POLLEN");
        flywheels[NECTAR] = hardwareMap.get(DcMotorEx.class, "NECTAR");
        intake = hardwareMap.get(DcMotorEx.class, "INTAKE");
        flywheels[POLLEN].setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        for (DcMotorEx motor : new DcMotorEx[]{flywheels[POLLEN], flywheels[NECTAR], intake}) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);              // required for setVelocity
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);    // coast when turned off
        }

        // Slide
        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "LS");
        slideMotor.setDirection(DcMotor.Direction.REVERSE);

        // Servos
        dropGate = hardwareMap.get(CRServo.class, "FLYWHEEL");   // the drop gate's config name in the old code
        flowerGate = hardwareMap.get(CRServo.class, "FLOWER");
        flowerGate.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareMap.get(CRServo.class, "UNLOCK").setPower(unlockpos);
        setDropGate(false);                                      // start with the balls held in the chamber
        setPowerIfChanged(flowerGate, SLOT_FLOWER_GATE, flowerClose);

        // Slide PID (slide must be fully DOWN at init)
        slide = new LinearSlideController(slideMotor);
        slide.init();

        // Bulk reads: one hub message per loop returns every encoder and velocity.
        // Turned on after slide.init() so encoder reads during init are always fresh.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    // =====================================================================
    //  BALL LIST (gamepad 2)
    // =====================================================================
    // Colour letters in the order the operator pressed them: 'Y' pollen, 'R' red nectar, 'B' blue nectar.
    private final char[] ballList = new char[CHAMBER_SIZE];
    private int ballCount = 0;

    private void readOperator() {
        if (justPressed(OP_POLLEN)) addBall('Y');
        if (justPressed(OP_RED_NECTAR)) addBall('R');
        if (justPressed(OP_BLUE_NECTAR)) addBall('B');

        if (justPressed(OP_UNDO) && ballCount > 0) {
            ballCount--;
            ballList[ballCount] = '-';
        }
        if (justPressed(OP_CLEAR)) {
            clearBallList();
        }
    }

    private void addBall(char colour) {
        if (ballCount < CHAMBER_SIZE) {
            ballList[ballCount] = colour;
            ballCount++;
            gamepad2.rumble(100);     // short buzz: accepted
        } else {
            gamepad2.rumble(600);     // long buzz: already 4 balls, ignored
        }
    }

    private void clearBallList() {
        Arrays.fill(ballList, '-');
        ballCount = 0;
    }

    private void removeFromList(int index) {
        for (int i = index; i < ballCount - 1; i++) {
            ballList[i] = ballList[i + 1];
        }
        ballCount--;
        ballList[ballCount] = '-';
    }

    private static int typeOf(char colour) {
        return (colour == 'Y') ? POLLEN : NECTAR;
    }

    */
/** Where in ballList is the ball at this position from the drop gate? (0 = touching the gate) *//*

    private int listIndexFromGate(int positionFromGate) {
        if (FIRST_IN_AT_GATE) {
            return positionFromGate;
        }
        return ballCount - 1 - positionFromGate;
    }

    // =====================================================================
    //  DRIVE + INTAKE
    // =====================================================================
    private boolean driveFlipped = false;
    private final double[] drivePower = new double[4];   // LB, LF, RB, RF

    private void updateDrive() {
        if (justPressed(BTN_RIGHT_BUMPER)) {
            driveFlipped = !driveFlipped;
        }

        // Flipping the front reverses forward and strafe, but not turning.
        double flipSign = driveFlipped ? -1 : 1;
        double forward = flipSign * gamepad1.left_stick_y;
        double strafe = -flipSign * gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        drivePower[0] = forward - strafe + turn;   // LB
        drivePower[1] = forward + strafe + turn;   // LF
        drivePower[2] = forward + strafe - turn;   // RB
        drivePower[3] = forward - strafe - turn;   // RF

        // If any wheel would go over 1.0, scale all four down together to keep the direction.
        double largest = 1.0;
        for (double power : drivePower) {
            largest = Math.max(largest, Math.abs(power));
        }
        for (int i = 0; i < 4; i++) {
            setPowerIfChanged(driveMotors[i], i, drivePower[i] / largest);
        }
    }

    private void updateIntake() {
        double speed = 0;
        if (gamepad1.left_trigger > 0.1) {
            speed = INTAKE_VEL;
        } else if (gamepad1.left_bumper) {
            speed = -INTAKE_VEL;
        }
        setVelocityIfChanged(intake, SLOT_INTAKE, speed);
    }

    // =====================================================================
    //  FLYWHEELS + SHOT DETECTION
    // =====================================================================
    private final boolean[] wheelOn = new boolean[2];
    private final boolean[] wheelArmed = new boolean[2];   // wheel reached speed, watching for a dip
    private final double[] wheelSpeed = new double[2];     // measured, ticks per second
    private final int[] shotsFired = new int[2];           // shots detected on each wheel

    private double targetSpeed(int wheel) {
        return (wheel == NECTAR) ? NECTAR_VEL : POLLEN_VEL;
    }

    */
/** Wheel is on and within READY_TOL of its target speed. *//*

    private boolean atSpeed(int wheel) {
        if (!wheelOn[wheel]) {
            return false;
        }
        double target = targetSpeed(wheel);
        return Math.abs(wheelSpeed[wheel] - target) <= target * READY_TOL;
    }

    private void updateFlywheels() {
        if (justPressed(BTN_B)) wheelOn[NECTAR] = true;
        if (justPressed(BTN_A)) wheelOn[POLLEN] = true;
        if (justPressed(BTN_X)) {
            wheelOn[POLLEN] = false;
            wheelOn[NECTAR] = false;
            if (step != Step.IDLE) {
                finishRound("STOPPED (X) - re-enter balls on gamepad 2");
            }
        }

        // Shot detection: a wheel at speed is "armed". If an armed wheel suddenly slows
        // down by DIP_FRAC, a ball just went through it, so count one shot.
        for (int wheel = 0; wheel < 2; wheel++) {
            wheelSpeed[wheel] = flywheels[wheel].getVelocity();
            if (!wheelOn[wheel]) {
                wheelArmed[wheel] = false;
            } else if (atSpeed(wheel)) {
                wheelArmed[wheel] = true;
            } else if (wheelArmed[wheel] && wheelSpeed[wheel] < targetSpeed(wheel) * (1 - DIP_FRAC)) {
                wheelArmed[wheel] = false;
                shotsFired[wheel]++;
            }
        }
    }

    private void sendFlywheelSpeeds() {
        for (int wheel = 0; wheel < 2; wheel++) {
            double speed = wheelOn[wheel] ? targetSpeed(wheel) : 0;
            setVelocityIfChanged(flywheels[wheel], SLOT_FLYWHEEL + wheel, speed);
        }
    }

    // =====================================================================
    //  SMART SHOOT (RT)
    // =====================================================================
    //  IDLE --RT--> WAIT <--> FEED --last ball--> IDLE
    //  WAIT: drop gate closed, the current ball waits for its flywheel (this is also the spin-up).
    //  FEED: drop gate open, the current ball is rolling through it.
    private enum Step { IDLE, WAIT, FEED }

    private Step step = Step.IDLE;
    private final int[] roundOrder = new int[CHAMBER_SIZE];   // this round's ball types, gate first
    private int roundSize;
    private int currentBall;
    private final int[] shotsAtRelease = new int[2];          // per wheel: shot count when its latest ball was released
    private final double[] lastPassTime = new double[2];      // per wheel: when its latest ball passed the gate
    private boolean startedFromRest;
    private boolean dropGateOpen = false;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private String lastResult = "ready";

    private void goTo(Step next) {
        step = next;
        stepTimer.reset();
    }

    private void setDropGate(boolean open) {
        dropGateOpen = open;
        setPowerIfChanged(dropGate, SLOT_DROP_GATE, open ? DROP_OPEN : DROP_CLOSED);
    }

    */
/** An unknown ball might go to either wheel, so it "uses" both. *//*

    private static boolean usesWheel(int ballType, int wheel) {
        return ballType == UNKNOWN || ballType == wheel;
    }

    private void updateShooting() {
        switch (step) {
            case IDLE:
                if (justPressed(BTN_RIGHT_TRIGGER) && !flowerPulseActive) {
                    startRound();
                }
                break;
            case WAIT:
                waitForWheel();
                break;
            case FEED:
                feedBall();
                break;
        }
    }

    private void startRound() {
        flowerLatchedOpen = false;
        setPowerIfChanged(flowerGate, SLOT_FLOWER_GATE, flowerClose);   // never shoot with the flower gate open

        if (ballCount > 0) {
            // Use the operator's list, starting with the ball at the gate.
            roundSize = ballCount;
            for (int i = 0; i < ballCount; i++) {
                roundOrder[i] = typeOf(ballList[listIndexFromGate(i)]);
            }
            lastResult = "firing " + ballCount;
        } else {
            // Nothing entered: assume a full chamber of unknowns, with full pauses. Slow but safe.
            roundSize = CHAMBER_SIZE;
            Arrays.fill(roundOrder, UNKNOWN);
            lastResult = "BLIND (nothing entered)";
        }
        clearBallList();   // these balls are leaving; anything entered from now on is a new ball

        // Turn on only the wheels this round needs (wheels already on stay on).
        for (int i = 0; i < roundSize; i++) {
            if (usesWheel(roundOrder[i], POLLEN)) wheelOn[POLLEN] = true;
            if (usesWheel(roundOrder[i], NECTAR)) wheelOn[NECTAR] = true;
        }

        currentBall = 0;
        setDropGate(false);
        goTo(Step.WAIT);
    }

    */
/** Gate closed. Open it once the current ball's wheel is ready, or once we've waited too long. *//*

    private void waitForWheel() {
        double waited = stepTimer.milliseconds();
        double timeLimit = (currentBall == 0) ? SPIN_TIMEOUT_MS : HOLD_MAX_MS;
        boolean gateFullyClosed = waited >= GATE_MIN_MS;

        if ((gateFullyClosed && currentBallMayGo()) || waited >= timeLimit) {
            setDropGate(true);
            releaseBall(true);   // the ball was sitting at the closed gate, so it starts from rest
        }
    }

    */
/** Gate open. Once the ball has passed, either let the next one roll on or close the gate. *//*

    private void feedBall() {
        int type = roundOrder[currentBall];
        double passTime = (type == POLLEN) ? POLLEN_CLEAR_MS : NECTAR_CLEAR_MS;
        if (startedFromRest) {
            passTime += FROM_REST_EXTRA_MS;
        }
        if (stepTimer.milliseconds() < passTime) {
            return;   // ball is still passing the gate
        }

        // The ball has passed the gate: note the time against its wheel(s).
        for (int wheel = 0; wheel < 2; wheel++) {
            if (usesWheel(type, wheel)) {
                lastPassTime[wheel] = clock.milliseconds();
            }
        }

        currentBall++;
        if (currentBall >= roundSize) {
            finishRound("done, fed " + roundSize);      // last ball is through: close up
        } else if (currentBallMayGo()) {
            releaseBall(false);                         // wheel ready: gate stays open, ball rolls straight on
        } else {
            setDropGate(false);                         // wheel still recovering: close the gate = the pause
            goTo(Step.WAIT);
        }
    }

    */
/** Start feeding the current ball. Remember each wheel's shot count so we can tell when THIS ball fires. *//*

    private void releaseBall(boolean fromRest) {
        startedFromRest = fromRest;
        for (int wheel = 0; wheel < 2; wheel++) {
            if (usesWheel(roundOrder[currentBall], wheel)) {
                shotsAtRelease[wheel] = shotsFired[wheel];
            }
        }
        goTo(Step.FEED);
    }

    */
/**
     * THE PAUSE RULE. May the current ball go now?
     * For each wheel it needs, that wheel must have recovered from its previous ball:
     *   (the previous ball was seen firing OR enough time has passed) AND the wheel is back at speed.
     * Two of the same type in a row are therefore always paused, and the pause ends as soon as
     * the wheel has really recovered. For the first ball this is simply "wheel at speed".
     *//*

    private boolean currentBallMayGo() {
        int type = roundOrder[currentBall];
        double now = clock.milliseconds();

        for (int wheel = 0; wheel < 2; wheel++) {
            if (!usesWheel(type, wheel)) {
                continue;
            }
            double maxPause = (wheel == POLLEN) ? PP_HOLD_MS : NN_HOLD_MS;
            boolean waitedLongEnough = now - lastPassTime[wheel] >= maxPause;
            boolean previousBallFired = shotsFired[wheel] > shotsAtRelease[wheel];

            boolean wheelReady;
            if (USE_VEL_READY) {
                wheelReady = (waitedLongEnough || previousBallFired) && atSpeed(wheel);
            } else {
                wheelReady = waitedLongEnough;
            }
            if (!wheelReady) {
                return false;
            }
        }
        return true;
    }

    private void finishRound(String result) {
        setDropGate(false);
        step = Step.IDLE;
        lastResult = result;
    }

    // =====================================================================
    //  MANUAL GATES + FLOWER DROP
    // =====================================================================
    private boolean flowerLatchedOpen = false;
    private boolean flowerPulseActive = false;
    private double flowerOpenMs;
    private int lastDropType = EMPTY;      // for telemetry
    private int lastBehindType = EMPTY;    // for telemetry
    private final ElapsedTime flowerTimer = new ElapsedTime();

    private void updateGates() {
        // Close the flower gate when a single drop's time is up.
        if (flowerPulseActive && flowerTimer.milliseconds() >= flowerOpenMs) {
            setPowerIfChanged(flowerGate, SLOT_FLOWER_GATE, flowerClose);
            flowerPulseActive = false;
        }

        if (step != Step.IDLE) {
            return;   // smart shoot controls the drop gate while it runs
        }

        if (justPressed(BTN_Y)) {
            setDropGate(!dropGateOpen);   // manual drop gate (does not change the ball list)
        }
        if (justPressed(BTN_DPAD_RIGHT) && !flowerPulseActive) {
            flowerLatchedOpen = !flowerLatchedOpen;
            setPowerIfChanged(flowerGate, SLOT_FLOWER_GATE, flowerLatchedOpen ? flowerOpen : flowerClose);
        }
        if (justPressed(BTN_DPAD_DOWN) && !flowerPulseActive && !flowerLatchedOpen) {
            dropOneIntoFlower();
        }
    }

    */
/** Open the flower gate for exactly one ball. The time depends on that ball and the one behind it. *//*

    private void dropOneIntoFlower() {
        int dropPosition = FLOWER_FROM_FAR_END ? ballCount - 1 : 0;     // the ball at the flower gate
        int behindPosition = FLOWER_FROM_FAR_END ? ballCount - 2 : 1;   // the ball behind it

        if (ballCount == 0) {
            // Nothing in the list: drop anyway (the driver asked) with the safe timing.
            lastDropType = UNKNOWN;
            lastBehindType = UNKNOWN;
        } else {
            lastDropType = typeOf(ballList[listIndexFromGate(dropPosition)]);
            if (ballCount > 1) {
                lastBehindType = typeOf(ballList[listIndexFromGate(behindPosition)]);
            } else {
                lastBehindType = EMPTY;
            }
            removeFromList(listIndexFromGate(dropPosition));
        }

        flowerOpenMs = Math.max(50, flowerBaseTime(lastDropType) + flowerTrim(lastBehindType));
        setPowerIfChanged(flowerGate, SLOT_FLOWER_GATE, flowerOpen);
        flowerTimer.reset();
        flowerPulseActive = true;
    }

    private double flowerBaseTime(int dropType) {
        if (dropType == POLLEN) return FLOWER_POLLEN_MS;
        if (dropType == NECTAR) return FLOWER_NECTAR_MS;
        return FLOWER_UNKNOWN_MS;
    }

    private double flowerTrim(int behindType) {
        if (behindType == POLLEN) return FLOWER_NEXT_POLLEN_TRIM_MS;
        if (behindType == NECTAR) return FLOWER_NEXT_NECTAR_TRIM_MS;
        if (behindType == EMPTY) return FLOWER_LAST_EXTRA_MS;
        return 0;   // unknown: no adjustment
    }

    // =====================================================================
    //  TELEMETRY (10 times a second)
    // =====================================================================
    private final StringBuilder chamberText = new StringBuilder(CHAMBER_SIZE);

    private void updateTelemetry() {
        if (telemetryTimer.milliseconds() < 100) {
            return;
        }
        telemetryTimer.reset();

        // Ball list, gate end first.
        chamberText.setLength(0);
        for (int position = 0; position < CHAMBER_SIZE; position++) {
            if (position < ballCount) {
                chamberText.append(ballList[listIndexFromGate(position)]);
            } else {
                chamberText.append('-');
            }
        }

        String stepInfo = (step == Step.IDLE) ? lastResult : "ball " + (currentBall + 1) + "/" + roundSize;
        String flowerState = flowerLatchedOpen ? "OPEN" : (flowerPulseActive ? "PULSE" : "closed");

        telemetry.addData("Seq", "%s %s", step.name(), stepInfo);
        telemetry.addData("Chamber gate>end", "[%s]  drop gate %s", chamberText, dropGateOpen ? "OPEN" : "closed");
        telemetry.addData("Nectar", "%.0f/%.0f %s fired %d", wheelSpeed[NECTAR], wheelOn[NECTAR] ? NECTAR_VEL : 0,
                atSpeed(NECTAR) ? "RDY" : "--", shotsFired[NECTAR]);
        telemetry.addData("Pollen", "%.0f/%.0f %s fired %d", wheelSpeed[POLLEN], wheelOn[POLLEN] ? POLLEN_VEL : 0,
                atSpeed(POLLEN) ? "RDY" : "--", shotsFired[POLLEN]);
        telemetry.addData("Flower", "%s %c>%c %.0fms", flowerState,
                TYPE_LETTERS.charAt(lastDropType), TYPE_LETTERS.charAt(lastBehindType), flowerOpenMs);
        telemetry.addData("Drive", "%s | loop %.1f ms", driveFlipped ? "FLIPPED" : "normal", loopMs);
        telemetry.update();
    }
}
*/
