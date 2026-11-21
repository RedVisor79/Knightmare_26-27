package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shared time-based drive code for all autos.
 * Robot starts with its back against the wall, facing into the field.
 *
 * Motors run in RUN_USING_ENCODER (velocity control), so speed stays consistent
 * as the battery drains. Drive encoders must be plugged in (same as TeleOp).
 *
 * Because the field is rotationally symmetric, every auto works on BOTH alliances:
 * just place the robot in the matching spot on the other side.
 */
@Config
public abstract class AutoBase extends LinearOpMode {

    // ---- Tune in FTC Dashboard (get speeds from the "Drive Calibration" auto) ----
    public static double DRIVE_POWER = 0.5;        // keep <= ~0.7 so velocity control has headroom
    public static double FORWARD_IN_PER_SEC = 25;  // inches/sec driving forward at DRIVE_POWER
    public static double STRAFE_IN_PER_SEC = 20;   // inches/sec strafing at DRIVE_POWER
    public static boolean FLIP_FRONT = false;      // set true if "forward" drives into the wall
    public static long START_DELAY_MS = 0;         // wait before moving (stay out of partner's way)
    public static long SETTLE_MS = 250;            // pause between moves

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
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // stop where the timer ends
        }

        telemetry.addData("Status", "Ready: back against wall, facing into field");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        pause(START_DELAY_MS);
        runPath();
        stopDrive();
    }

    // ---- Moves (inches, robot-relative) ----
    protected void forward(double inches)     { drive(Math.signum(inches), 0, Math.abs(inches) / FORWARD_IN_PER_SEC); }
    protected void backward(double inches)    { forward(-inches); }
    protected void strafeRight(double inches) { drive(0, Math.signum(inches), Math.abs(inches) / STRAFE_IN_PER_SEC); }
    protected void strafeLeft(double inches)  { strafeRight(-inches); }

    /** Drive for a fixed time. f/s are directions (-1..1): +f = forward, +s = right. */
    protected void drive(double f, double s, double seconds) {
        if (FLIP_FRONT) { f = -f; s = -s; }
        double fwd = f * DRIVE_POWER;
        double str = s * DRIVE_POWER;

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
            telemetry.addData("Moving", "f=%.1f s=%.1f  %.2f / %.2f s", f, s, timer.seconds(), seconds);
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