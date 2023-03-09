package org.firstinspires.ftc.teamcode.cog.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.cog.component.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Robot;


public class RobotOpMode extends OpMode {
    // Instance Variables
    protected Robot robot;
    protected Drivetrain drivetrain;

    @Override
    public void init() {
        this.robot = new Robot(hardwareMap, telemetry);
        this.drivetrain = robot.drivetrain;
    }

    @Override
    public void loop() {
        robot.update();
    }

}