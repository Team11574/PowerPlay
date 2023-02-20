package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

public abstract class RobotLinearOpMode extends LinearOpMode {
    // Instance Variables
    protected Robot robot;

    public RobotLinearOpMode() {
        this.robot = new Robot(hardwareMap, telemetry);
    }
}
