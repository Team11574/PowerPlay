package org.firstinspires.ftc.teamcode.opmodes.base;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.Drivetrain;

public abstract class RobotLinearOpMode extends LinearOpMode {
    // Instance Variables
    protected Robot robot;
    protected Drivetrain drivetrain;

    @Override
    public void runOpMode() {
        this.robot = new Robot(hardwareMap, telemetry);
    }
}
