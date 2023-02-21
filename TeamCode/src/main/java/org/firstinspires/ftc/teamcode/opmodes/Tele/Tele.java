package org.firstinspires.ftc.teamcode.opmodes.Tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.Drivetrain;

@TeleOp(name = "Tele", group = "tele")
public class Tele extends OpMode {
    // Instance Variables
    protected Robot robot;
    protected Drivetrain drivetrain;

    @Override
    public void init() {
        this.robot = new Robot(hardwareMap, telemetry);

        drivetrain = robot.getDrivetrain();
    }

    @Override
    public void loop() {
        double velY = -gamepad1.left_stick_y;
        double velX = gamepad1.left_stick_x;
        double theta = gamepad1.right_stick_x;

        double normalFactor = Math.max(Math.abs(velY) + Math.abs(velX) + Math.abs(theta), 1);
        double frontRight_Power = (velY - velX - theta) / normalFactor;
        double backRight_Power = (velY + velX - theta) / normalFactor;
        double frontLeft_Power = (velY + velX + theta) / normalFactor;
        double backLeft_Power = (velY - velX + theta) / normalFactor;

        drivetrain.setMotorPowers(frontLeft_Power, backLeft_Power, backRight_Power, frontRight_Power);


    }
}
