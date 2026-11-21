package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shared time-based drive code for all autos (not an OpMode itself).
 * Robot starts with its back against the wall, facing into the field.
 * Field is rotationally symmetric, so each auto works on BOTH alliances
 * from the matching start spot.
 */
@Config
public abstract class AutoBase extends LinearOpMode {

    public static double DRIVE_POWER = 0.08;   // tuned on Primary; shared by every auto
    public static boolean FLIP_FRONT = true;   // tuned: robot's front is the TeleOp "reverse" side
    public static long START_DELAY_MS = 0;     // wait before moving (stay out of partner's way)
    public static long SETTLE_MS = 250;        // pause after each move

    private DcMotor LB, LF, RB, RF;

    /** Each auto's path goes here. */
    protected abstract void runPath();

    @Override
    public void runOpMode() {
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");

        LB.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{LB, LF, RB, RF}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addData("Status", "Ready: back against wall, facing into field");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        pause(START_DELAY_MS);
        runPath();
        stopDrive();
    }

    // ---- Moves at DRIVE_POWER for a set number of seconds ----
    protected void forward(double seconds)     { drive(DRIVE_POWER, 0, seconds); }
    protected void backward(double seconds)    { drive(-DRIVE_POWER, 0, seconds); }
    protected void strafeRight(double seconds) { drive(0, DRIVE_POWER, seconds); }
    protected void strafeLeft(double seconds)  { drive(0, -DRIVE_POWER, seconds); }

    /** fwd/str are signed powers: +fwd = forward, +str = right. */
    protected void drive(double fwd, double str, double seconds) {
        if (FLIP_FRONT) { fwd = -fwd; str = -str; }

        // Same mecanum math as TeleOp (turn = 0)
        double lb = fwd - str;
        double lf = fwd + str;
        double rb = fwd + str;
        double rf = fwd - str;

        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lb), Math.abs(rb)));
        if (max > 1.0) { lb /= max; lf /= max; rb /= max; rf /= max; }

        LB.setPower(lb);
        LF.setPower(lf);
        RB.setPower(rb);
        RF.setPower(rf);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            telemetry.addData("Moving", "%.2f / %.2f s", timer.seconds(), seconds);
            telemetry.update();
            idle();
        }

        stopDrive();
        pause(SETTLE_MS);
    }

    protected void stopDrive() {
        LB.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        RF.setPower(0);
    }

    protected void pause(long ms) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < ms) idle();
    }
}