package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * SECONDARY (purple path)
 * Red:  back against the BOTTOM wall, ~15" right of the red box, facing up.
 * Blue: back against the TOP wall, ~15" left of the blue box, facing down (same spot rotated 180°).
 * Drives forward, then strafes left to stop just outside the alliance box corner.
 */
@Config
@Autonomous(name = "Secondary", group = "Auto")
public class Secondary extends AutoBase {

    public static double SECONDARY_FORWARD_IN = 17;
    public static double SECONDARY_LEFT_IN = 15;

    @Override
    protected void runPath() {
        forward(SECONDARY_FORWARD_IN);
        strafeLeft(SECONDARY_LEFT_IN);
    }
}