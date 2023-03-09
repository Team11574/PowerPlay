package org.firstinspires.ftc.teamcode.cog.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.cog.component.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Robot;

public abstract class RobotLinearOpMode extends LinearOpMode {
    // Instance Variables
    protected Robot robot;
    protected Drivetrain drivetrain;

    @Override
    public void runOpMode() {
        this.robot = new Robot(hardwareMap, telemetry);
    }
}