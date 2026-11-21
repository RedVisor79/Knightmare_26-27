package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Run once (in open space) to find FORWARD_IN_PER_SEC and STRAFE_IN_PER_SEC.
 * 1. Robot drives forward for CAL_SECONDS. Measure inches traveled -> divide by CAL_SECONDS.
 * 2. After a pause, it strafes RIGHT for CAL_SECONDS. Measure again -> divide by CAL_SECONDS.
 * Put the results into AutoBase in Dashboard (and in code so they stick).
 * Also confirms directions: if it drives backward, set FLIP_FRONT = true.
 */
@Config
@Autonomous(name = "Drive Calibration", group = "Auto")
public class DriveCalibration extends AutoBase {

    public static double CAL_SECONDS = 2.0;
    public static long MEASURE_PAUSE_MS = 5000;

    @Override
    protected void runPath() {
        drive(1, 0, CAL_SECONDS);
        telemetry.addLine("Measure FORWARD distance. FORWARD_IN_PER_SEC = inches / " + CAL_SECONDS);
        telemetry.update();
        pause(MEASURE_PAUSE_MS);

        drive(0, 1, CAL_SECONDS);
        telemetry.addLine("Measure STRAFE distance. STRAFE_IN_PER_SEC = inches / " + CAL_SECONDS);
        telemetry.update();
        pause(MEASURE_PAUSE_MS);
    }
}