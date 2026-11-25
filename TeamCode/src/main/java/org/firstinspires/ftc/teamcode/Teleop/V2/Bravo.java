package org.firstinspires.ftc.teamcode.Teleop.V2;

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

import org.firstinspires.ftc.teamcode.Teleop.V1.LinearSlideController;

import java.util.Arrays;
import java.util.List;

/*
 * GIMP TeleOp — BIOBUZZ, two controllers.
 *
 * HOW A BALL MOVES, FROM INTAKE TO LAUNCH
 *   1. The intake pulls a ball into the slide chamber. The rails slope to the LEFT, so the ball
 *      rolls left until it rests against the drop gate, or against the ball already in front of it.
 *   2. The operator presses that ball's colour on gamepad 2. The ball list always matches the
 *      chamber: slot 0 = the ball touching the drop gate, slot 3 = the back of the chamber.
 *   3. RT: the flywheels spin up, then the drop gate opens. Balls drop forward and down into
 *      the passive sorter, which sends nectar to the nectar flywheel and pollen to the pollen one.
 *   4. Every time a ball leaves the chamber (launched, or dropped into a flower), the list
 *      moves forward one slot: slot 1 -> 0, slot 2 -> 1, slot 3 -> 2, and slot 3 becomes empty.
 *
 * THE SLIDE
 *   The ball chamber rides up with the linear slide. Balls can only reach the sorter when the
 *   chamber is DOWN, so RT and opening the drop gate only work with the slide down, and the
 *   slide can't be moved while a round is shooting.
 *
 * THE TWO TIMED GATES
 *   Drop gate:   stays open for the whole round. It only closes for a short pause when the next
 *                ball is the SAME size as the one just launched (two pollen, or two nectar), so
 *                that flywheel has time to get back up to speed.
 *   Flower gate: single drop (dpad_down) opens it just long enough for the ball in slot 0,
 *                using that ball's own time (pollen time or nectar time).
 *
 * GAMEPAD 2 (operator)  Y pollen | B red nectar | X blue nectar | LB undo last | BACK clear all
 * GAMEPAD 1 (driver)    sticks drive | RB flip front | B / A nectar / pollen wheel on
 *                       X wheels off + stop shooting | LT / LB intake in / out | RT shoot
 *                       Y drop gate open/close | dpad_up slide | dpad_right flower gate open/close
 *                       dpad_down drop one ball into a flower
 *
 * Every "public static" value can be changed live in FTC Dashboard (192.168.43.1:8080/dash).
 * Dashboard changes are lost on restart, so copy good values back into this file.
 */
@Config
@TeleOp(name = "Bravo")
public class Bravo extends LinearOpMode {

    // =====================================================================
    //  TUNABLES
    // =====================================================================

    // Flywheels and intake (encoder ticks per second).
    // Flywheel speeds must be reachable, or "at speed" never comes true and every round
    // waits the full SPIN_UP_TIMEOUT_MS. After changing a speed in Dashboard, press B / A again.
    public static double NECTAR_VEL = 6000;
    public static double POLLEN_VEL = 6000;
    public static double INTAKE_VEL = 100;
    public static double READY_TOL = 0.05;            // within 5% of target = "at speed"
    public static double SPIN_UP_TIMEOUT_MS = 2000;   // shoot anyway if the wheels never read "at speed"

    // Drop gate positions (these servos are configured as CRServo, so power = position)
    public static double DROP_CLOSED = 0.35;          // holds the balls in the chamber
    public static double DROP_OPEN = -0.95;           // balls drop into the sorter. Swap the two if reversed.

    // Drop gate timing (milliseconds)
    public static double POLLEN_PASS_MS = 90;         // time for a pollen to roll fully past the drop gate
    public static double NECTAR_PASS_MS = 115;        // nectar is bigger, so it takes longer
    public static double FROM_STOP_EXTRA_MS = 80;     // extra time when the ball starts from a stop at the gate
    public static double POLLEN_PAIR_DELAY_MS = 350;  // gate stays shut this long between two pollen in a row
    public static double NECTAR_PAIR_DELAY_MS = 450;  // ...and between two nectar in a row (heavier ball)

    // Flower gate
    public static double flowerClose = 0;
    public static double flowerOpen = 0.75;
    public static double FLOWER_POLLEN_MS = 430;      // single drop: open time for a pollen
    public static double FLOWER_NECTAR_MS = 550;      // single drop: open time for a nectar (also used if the list is empty)

    public static double unlockpos = 0.15;

    // Slide: the chamber counts as "down" when the slide encoder is within this many ticks of
    // where it was at init (the slide must be fully down at init). Check the "Slide" telemetry line to tune.
    public static int SLIDE_DOWN_TICKS = 50;

    // =====================================================================
    //  HARDWARE
    // =====================================================================
    private final DcMotor[] driveMotors = new DcMotor[4];   // LB, LF, RB, RF
    private DcMotorEx pollenWheel;                          // config "POLLEN"
    private DcMotorEx nectarWheel;                          // config "NECTAR"
    private DcMotorEx intake;                               // config "INTAKE"
    private CRServo dropGate;                               // config "FLYWHEEL"
    private CRServo flowerGate;                             // config "FLOWER"
    private DcMotor slideMotor;                             // config "LS"
    private int slideDownPosition;                          // slide encoder reading at init (fully down)
    private LinearSlideController slide;                    // moves slideMotor up / down
    private List<LynxModule> hubs;

    // =====================================================================
    //  GAMEPADS
    // =====================================================================
    // A copy of each gamepad from this loop and from the last loop. A button was "just pressed"
    // when it is down now but was up last loop, so holding a button only acts once.
    private final Gamepad driver = new Gamepad();
    private final Gamepad lastDriver = new Gamepad();
    private final Gamepad operator = new Gamepad();
    private final Gamepad lastOperator = new Gamepad();

    private static boolean pressed(boolean downNow, boolean downLastLoop) {
        return downNow && !downLastLoop;
    }

    // =====================================================================
    //  MAIN LOOP
    // =====================================================================
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private double loopMs;

    @Override
    public void runOpMode() {
        initHardware();
        telemetry.addLine("GAMEPAD 2: Y pollen | B red nectar | X blue nectar | LB undo | BACK clear");
        telemetry.update();

        waitForStart();
        slide.start();

        while (opModeIsActive()) {
            loopMs = loopTimer.milliseconds();
            loopTimer.reset();
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();   // fresh encoder and speed readings for this loop
            }
            lastDriver.copy(driver);
            driver.copy(gamepad1);
            lastOperator.copy(operator);
            operator.copy(gamepad2);

            readOperator();           // gamepad 2 keeps the ball list up to date
            drive();
            runIntake();
            slide.update(driver.dpad_up && shootStep == ShootStep.IDLE);   // no slide moves mid-round
            flywheelButtons();        // B / A / X
            updateShooting();         // RT and the drop gate
            updateFlowerAndGates();   // flower gate, single drop, manual drop gate
            showTelemetry();
        }
    }

    private void initHardware() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        String[] driveNames = {"LB", "LF", "RB", "RF"};
        for (int i = 0; i < 4; i++) {
            driveMotors[i] = hardwareMap.get(DcMotor.class, driveNames[i]);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        driveMotors[0].setDirection(DcMotor.Direction.REVERSE);   // LB
        driveMotors[1].setDirection(DcMotor.Direction.REVERSE);   // LF

        pollenWheel = hardwareMap.get(DcMotorEx.class, "POLLEN");
        nectarWheel = hardwareMap.get(DcMotorEx.class, "NECTAR");
        intake = hardwareMap.get(DcMotorEx.class, "INTAKE");
        pollenWheel.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        for (DcMotorEx motor : new DcMotorEx[]{pollenWheel, nectarWheel, intake}) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);             // required for setVelocity
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);   // coast when turned off
        }

        slideMotor = hardwareMap.get(DcMotor.class, "LS");
        slideMotor.setDirection(DcMotor.Direction.REVERSE);

        dropGate = hardwareMap.get(CRServo.class, "FLYWHEEL");
        flowerGate = hardwareMap.get(CRServo.class, "FLOWER");
        flowerGate.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareMap.get(CRServo.class, "UNLOCK").setPower(unlockpos);
        setDropGate(false);     // hold the balls in the chamber
        setFlowerGate(false);

        slide = new LinearSlideController(slideMotor);   // slide must be fully DOWN at init
        slide.init();
        slideDownPosition = slideMotor.getCurrentPosition();   // remember "down" (read before bulk caching is on)

        // Bulk reads: one message per hub per loop fetches every encoder and speed.
        // Turned on after slide.init() so encoder reads during init are always fresh.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    // =====================================================================
    //  BALL LIST (gamepad 2)
    // =====================================================================
    private enum Ball {
        EMPTY('-'), POLLEN('Y'), RED_NECTAR('R'), BLUE_NECTAR('B');

        final char letter;   // shown on the Driver Station; matches the gamepad 2 button colour

        Ball(char letter) {
            this.letter = letter;
        }

        boolean isNectar() {
            return this == RED_NECTAR || this == BLUE_NECTAR;
        }
    }

    private static final int CHAMBER_SIZE = 4;

    /** balls[0] touches the drop gate; balls[3] is at the back of the chamber. */
    private final Ball[] balls = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};

    private void readOperator() {
        if (pressed(operator.y, lastOperator.y)) addBall(Ball.POLLEN);        // yellow button
        if (pressed(operator.b, lastOperator.b)) addBall(Ball.RED_NECTAR);    // red button
        if (pressed(operator.x, lastOperator.x)) addBall(Ball.BLUE_NECTAR);   // blue button

        if (pressed(operator.right_bumper, lastOperator.right_bumper) && ballCount() > 0) {
            balls[ballCount() - 1] = Ball.EMPTY;   // undo: remove the last ball entered
        }
        if (pressed(operator.right_trigger>0, lastOperator.right_trigger>0)) {
            Arrays.fill(balls, Ball.EMPTY);        // clear the whole list
        }
    }

    /** A new ball rolled in behind the others, so it goes in the first empty slot. */
    private void addBall(Ball ball) {
        int slot = ballCount();
        if (slot == CHAMBER_SIZE) {
            gamepad2.rumble(600);   // long buzz: chamber already full, ignored
            return;
        }
        balls[slot] = ball;
        gamepad2.rumble(100);       // short buzz: accepted
        if (shootStep != ShootStep.IDLE) {
            turnOnWheelFor(ball);   // entered mid-round: start its flywheel now
        }
    }

    /** The ball in slot 0 has left the chamber: every ball moves one slot closer to the gate. */
    private void shiftForward() {
        for (int i = 0; i < CHAMBER_SIZE - 1; i++) {
            balls[i] = balls[i + 1];
        }
        balls[CHAMBER_SIZE - 1] = Ball.EMPTY;
    }

    private int ballCount() {
        int count = 0;
        while (count < CHAMBER_SIZE && balls[count] != Ball.EMPTY) {
            count++;
        }
        return count;
    }

    /** Same size = same flywheel. Red and blue nectar are the same size. */
    private static boolean sameSize(Ball a, Ball b) {
        if (a == Ball.EMPTY || b == Ball.EMPTY) {
            return false;
        }
        return a.isNectar() == b.isNectar();
    }

    // =====================================================================
    //  DRIVE + INTAKE
    // =====================================================================
    private boolean driveFlipped = false;

    private void drive() {
        if (pressed(driver.right_bumper, lastDriver.right_bumper)) {
            driveFlipped = !driveFlipped;
        }

        // Flipping the front reverses forward and strafe, but not turning.
        double flip = driveFlipped ? -1 : 1;
        double forward = flip * driver.left_stick_y;
        double strafe = -flip * driver.left_stick_x;
        double turn = driver.right_stick_x;

        double lb = forward - strafe + turn;
        double lf = forward + strafe + turn;
        double rb = forward + strafe - turn;
        double rf = forward - strafe - turn;

        // If any wheel would go over 1.0, scale all four down together to keep the direction.
        double largest = 1.0;
        largest = Math.max(largest, Math.abs(lb));
        largest = Math.max(largest, Math.abs(lf));
        largest = Math.max(largest, Math.abs(rb));
        largest = Math.max(largest, Math.abs(rf));
        driveMotors[0].setPower(lb / largest);
        driveMotors[1].setPower(lf / largest);
        driveMotors[2].setPower(rb / largest);
        driveMotors[3].setPower(rf / largest);
    }

    private double intakeSpeed = 0;

    private void runIntake() {
        double speed = 0;
        if (driver.left_trigger > 0.1) {
            speed = INTAKE_VEL;
        } else if (driver.right_bumper) {
            speed = -INTAKE_VEL;
        }
        if (speed != intakeSpeed) {   // only send a command to the hub when it changes
            intake.setVelocity(speed);
            intakeSpeed = speed;
        }
    }

    // =====================================================================
    //  FLYWHEELS
    // =====================================================================
    private boolean pollenOn = false;
    private boolean nectarOn = false;

    private void setPollenWheel(boolean on) {
        pollenOn = on;
        pollenWheel.setVelocity(on ? POLLEN_VEL : 0);
    }

    private void setNectarWheel(boolean on) {
        nectarOn = on;
        nectarWheel.setVelocity(on ? NECTAR_VEL : 0);
    }

    private void turnOnWheelFor(Ball ball) {
        if (ball == Ball.POLLEN && !pollenOn) setPollenWheel(true);
        if (ball.isNectar() && !nectarOn) setNectarWheel(true);
    }

    private boolean pollenReady() {
        return pollenOn && Math.abs(pollenWheel.getVelocity() - POLLEN_VEL) <= POLLEN_VEL * READY_TOL;
    }

    private boolean nectarReady() {
        return nectarOn && Math.abs(nectarWheel.getVelocity() - NECTAR_VEL) <= NECTAR_VEL * READY_TOL;
    }

    /** Every flywheel needed by a ball in the list is at speed. */
    private boolean wheelsReady() {
        for (Ball ball : balls) {
            if (ball == Ball.POLLEN && !pollenReady()) return false;
            if (ball.isNectar() && !nectarReady()) return false;
        }
        return true;
    }

    private void flywheelButtons() {
        if (pressed(driver.b, lastDriver.b)) setNectarWheel(true);
        if (pressed(driver.a, lastDriver.a)) setPollenWheel(true);
        if (pressed(driver.x, lastDriver.x)) {
            setPollenWheel(false);
            setNectarWheel(false);
            if (shootStep != ShootStep.IDLE) {
                endRound("stopped (X) - check the list");
            }
        }
    }

    // =====================================================================
    //  SHOOTING (RT)
    // =====================================================================
    //  IDLE --RT--> SPINNING_UP --wheels ready--> ROLLING --ball passed--> next ball?
    //     next ball is empty           -> close the gate, back to IDLE
    //     next ball is the SAME size   -> close the gate, PAUSED, then reopen -> ROLLING
    //     next ball is a different size -> gate stays open, ROLLING
    private enum ShootStep { IDLE, SPINNING_UP, ROLLING, PAUSED }

    private ShootStep shootStep = ShootStep.IDLE;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private boolean dropGateOpen = false;
    private boolean ballStartedFromStop = false;   // true for the first ball after the gate opens
    private Ball lastLaunched = Ball.EMPTY;
    private String shootMessage = "ready";

    private void goTo(ShootStep step) {
        shootStep = step;
        stepTimer.reset();
    }

    private void setDropGate(boolean open) {
        dropGateOpen = open;
        dropGate.setPower(open ? DROP_OPEN : DROP_CLOSED);
    }

    private void updateShooting() {
        double timeInStep = stepTimer.milliseconds();

        switch (shootStep) {
            case IDLE:
                boolean rtPressed = pressed(driver.right_trigger > 0.3, lastDriver.right_trigger > 0.3);
                if (rtPressed && !flowerDropping) {
                    startRound();
                }
                break;

            case SPINNING_UP:
                if (wheelsReady() || timeInStep >= SPIN_UP_TIMEOUT_MS) {
                    openGateForNextBall();
                }
                break;

            case ROLLING:
                if (timeInStep >= passTime(balls[0])) {
                    ballLaunched();
                }
                break;

            case PAUSED:
                if (timeInStep >= pairDelay(lastLaunched)) {
                    openGateForNextBall();
                }
                break;
        }
    }

    /** The chamber is down, so balls dropped through the gate land in the sorter. */
    private boolean slideIsDown() {
        return Math.abs(slideMotor.getCurrentPosition() - slideDownPosition) <= SLIDE_DOWN_TICKS;
    }

    private void startRound() {
        if (!slideIsDown()) {
            shootMessage = "lower the slide to shoot";
            return;
        }
        if (ballCount() == 0) {
            shootMessage = "nothing entered on gamepad 2";
            return;
        }
        flowerLatchedOpen = false;
        setFlowerGate(false);   // never shoot with the flower gate open
        for (Ball ball : balls) {
            turnOnWheelFor(ball);
        }
        setDropGate(false);
        shootMessage = "shooting";
        goTo(ShootStep.SPINNING_UP);
    }

    /** Open the drop gate. The ball in slot 0 was sitting at the gate, so it starts from a stop. */
    private void openGateForNextBall() {
        setDropGate(true);
        ballStartedFromStop = true;
        goTo(ShootStep.ROLLING);
    }

    /** How long the gate must stay open for this ball to roll fully past it. */
    private double passTime(Ball ball) {
        double time = ball.isNectar() ? NECTAR_PASS_MS : POLLEN_PASS_MS;
        if (ballStartedFromStop) {
            time += FROM_STOP_EXTRA_MS;
        }
        return time;
    }

    /** How long the gate stays shut between two balls of this size in a row. */
    private double pairDelay(Ball ball) {
        return ball.isNectar() ? NECTAR_PAIR_DELAY_MS : POLLEN_PAIR_DELAY_MS;
    }

    /** The ball in slot 0 has rolled past the gate into the sorter. */
    private void ballLaunched() {
        lastLaunched = balls[0];
        shiftForward();
        Ball next = balls[0];

        if (next == Ball.EMPTY) {
            endRound("done");                  // chamber is empty: close the gate
        } else if (sameSize(lastLaunched, next)) {
            setDropGate(false);                // same flywheel next: close the gate and pause
            goTo(ShootStep.PAUSED);
        } else {
            ballStartedFromStop = false;       // other flywheel next: gate stays open, ball rolls on
            goTo(ShootStep.ROLLING);
        }
    }

    private void endRound(String message) {
        setDropGate(false);
        shootMessage = message;
        goTo(ShootStep.IDLE);
    }

    // =====================================================================
    //  FLOWER GATE + MANUAL DROP GATE
    // =====================================================================
    private boolean flowerLatchedOpen = false;   // dpad_right: held open until pressed again
    private boolean flowerDropping = false;      // dpad_down: a single drop is in progress
    private double flowerOpenMs = 0;
    private Ball flowerBall = Ball.EMPTY;        // the ball being dropped
    private final ElapsedTime flowerTimer = new ElapsedTime();

    private void setFlowerGate(boolean open) {
        flowerGate.setPower(open ? flowerOpen : flowerClose);
    }

    private void updateFlowerAndGates() {
        // A single drop has had its time: close the gate and record the ball as gone.
        if (flowerDropping && flowerTimer.milliseconds() >= flowerOpenMs) {
            setFlowerGate(false);
            flowerDropping = false;
            if (flowerBall != Ball.EMPTY) {
                shiftForward();
            }
        }

        if (shootStep != ShootStep.IDLE || flowerDropping) {
            return;   // ignore gate buttons while shooting or while a single drop is happening
        }

        if (pressed(driver.y, lastDriver.y)) {
            // Manual drop gate (does not change the list). It can always close,
            // but only opens with the slide down, so balls land in the sorter.
            if (dropGateOpen) {
                setDropGate(false);
            } else if (slideIsDown()) {
                setDropGate(true);
            } else {
                shootMessage = "lower the slide to open the drop gate";
            }
        }
        if (pressed(driver.dpad_right, lastDriver.dpad_right)) {
            flowerLatchedOpen = !flowerLatchedOpen;
            setFlowerGate(flowerLatchedOpen);   // (does not change the list)
        }
        if (pressed(driver.dpad_down, lastDriver.dpad_down) && !flowerLatchedOpen) {
            dropOneIntoFlower();
        }
    }

    /** Open the flower gate just long enough for the ball in slot 0, timed for that ball's type. */
    private void dropOneIntoFlower() {
        flowerBall = balls[0];
        flowerOpenMs = (flowerBall == Ball.POLLEN) ? FLOWER_POLLEN_MS : FLOWER_NECTAR_MS;
        setFlowerGate(true);
        flowerTimer.reset();
        flowerDropping = true;
    }

    // =====================================================================
    //  TELEMETRY (10 times a second)
    // =====================================================================
    private void showTelemetry() {
        if (telemetryTimer.milliseconds() < 100) {
            return;
        }
        telemetryTimer.reset();

        StringBuilder chamber = new StringBuilder();
        for (Ball ball : balls) {
            chamber.append(ball.letter).append(' ');
        }
        String flowerState = flowerLatchedOpen ? "OPEN"
                : flowerDropping ? "dropping " + flowerBall.letter + " (" + (int) flowerOpenMs + " ms)"
                : "closed";

        telemetry.addData("Shooting", shootStep == ShootStep.IDLE ? shootMessage : shootStep.name());
        telemetry.addData("Chamber (gate first)", "%s  drop gate %s", chamber, dropGateOpen ? "OPEN" : "closed");
        telemetry.addData("Pollen wheel", "%.0f / %.0f %s", pollenWheel.getVelocity(),
                pollenOn ? POLLEN_VEL : 0, pollenReady() ? "READY" : "");
        telemetry.addData("Nectar wheel", "%.0f / %.0f %s", nectarWheel.getVelocity(),
                nectarOn ? NECTAR_VEL : 0, nectarReady() ? "READY" : "");
        telemetry.addData("Flower gate", flowerState);
        telemetry.addData("Slide", "%s (%d ticks from down)", slideIsDown() ? "down" : "UP",
                slideMotor.getCurrentPosition() - slideDownPosition);
        telemetry.addData("Drive", "%s | loop %.1f ms", driveFlipped ? "FLIPPED" : "normal", loopMs);
        telemetry.update();
        telemetry.update();
    }
}
