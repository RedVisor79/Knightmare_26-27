package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * PRIMARY (light blue path)
 * Red:  back against the LEFT wall, bottom row, facing right.
 * Blue: back against the RIGHT wall, top row, facing left (same spot rotated 180°).
 * Drives straight out and stops above the alliance box.
 */
@Config
@Autonomous(name = "Primary", group = "Auto")
public class Primary extends AutoBase {

    public static double PRIMARY_FORWARD_IN = 35;

    @Override
    protected void runPath() {
        forward(PRIMARY_FORWARD_IN);
    }
}