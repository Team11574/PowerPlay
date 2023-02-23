package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.base.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.component.Drivetrain;
import org.firstinspires.ftc.teamcode.util.GamepadPlus;

@TeleOp(name = "Tele", group = "tele")
public class Tele extends RobotOpMode {
    // Instance Variables
    protected Robot robot;
    protected Drivetrain drivetrain;
    GamepadPlus pad1;
    GamepadPlus pad2;

    @Override
    public void init() {
        super.init();
        pad1 = new GamepadPlus(gamepad1);
        pad2 = new GamepadPlus(gamepad2);
    }

    @Override
    public void loop() {
        super.loop();
        update();

        double velY = -pad1.gamepad.left_stick_y;
        double velX = pad1.gamepad.left_stick_x;
        double theta = pad1.gamepad.right_stick_x;

        double normalFactor = Math.max(Math.abs(velY) + Math.abs(velX) + Math.abs(theta), 1);
        double frontRight_Power = (velY - velX - theta) / normalFactor;
        double backRight_Power = (velY + velX - theta) / normalFactor;
        double frontLeft_Power = (velY + velX + theta) / normalFactor;
        double backLeft_Power = (velY - velX + theta) / normalFactor;

        drivetrain.setMotorPowers(frontLeft_Power, backLeft_Power, backRight_Power, frontRight_Power);

        // HOLD TRIGGER to flip
        if (pad2.right_trigger_pressed()) {
            robot.verticalFlip.flipDown();
        } else {
            robot.verticalFlip.flipUp();
        }

        // TOGGLE X to toggle back claw
        if (pad2.x_pressed()) {
            robot.horizontalClaw.toggle();
        }

        // TOGGLE B to toggle front claw
        if (pad2.b_pressed()) {
            robot.horizontalClaw.toggle();
        }

        // PRESS A to retract
        if (pad2.a_pressed()) {
            robot.retractArm();
        }

        robot.verticalSlide.setPower(pad2.get_partitioned_left_stick_y());

    }

    public void update() {
        pad1.update();
        pad2.update();
    }
}
