package org.firstinspires.ftc.teamcode.opmodes.base;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.component.Drivetrain;


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
