package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * SECONDARY: pull off the wall slightly, then strafe toward the alliance box.
 * Red:  back against the BOTTOM wall, right of the red box, facing into field.
 * Blue: back against the TOP wall, left of the blue box, facing into field.
 * Power = AutoBase.DRIVE_POWER (same as Primary).
 */
@Config
@Autonomous(name = "Secondary", group = "Auto")
public class Secondary extends AutoBase {

    public static double OFF_WALL_SEC = 1.0;    // tuned: move up so it clears the wall
    public static double STRAFE_SEC = 1.5;      // tuned: strafe toward the box
    public static boolean STRAFE_RIGHT = false; // flip if it strafes away from the box

    @Override
    protected void runPath() {
        forward(OFF_WALL_SEC);
        if (STRAFE_RIGHT) strafeRight(STRAFE_SEC);
        else              strafeLeft(STRAFE_SEC);
    }
}