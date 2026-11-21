package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * PRIMARY: drive straight out from the wall.
 * Red:  back against the LEFT wall, bottom row, facing into field.
 * Blue: back against the RIGHT wall, top row, facing into field.
 * Power = AutoBase.DRIVE_POWER.
 */
@Config
@Autonomous(name = "Primary", group = "Auto")
public class Primary extends AutoBase {

    public static double PRIMARY_SEC = 1.25;   // tuned

    @Override
    protected void runPath() {
        forward(PRIMARY_SEC);
    }
}