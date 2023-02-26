package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.opmodes.base.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.component.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.component.slide.VerticalSlide;
import org.firstinspires.ftc.teamcode.util.GamepadPlus;

@TeleOp(name = "Tele", group = "tele")
public class Tele extends RobotOpMode {
    // Instance Variables
    protected Robot robot;
    protected Drivetrain drivetrain;
    GamepadPlus pad1;
    GamepadPlus pad2;
    MultipleTelemetry multiTelemetry;

    boolean goingPos0 = false;
    boolean goingPos1 = false;
    boolean goingPos2 = false;
    boolean goingPos3 = false;

    @Override
    public void init() {
        //super.init();
        this.robot = new Robot(hardwareMap, telemetry);
        robot.verticalClaw.open();
        this.drivetrain = robot.drivetrain;
        pad1 = new GamepadPlus(gamepad1);
        pad2 = new GamepadPlus(gamepad2);
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        //super.loop();
        robot.update();
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
        if (!robot.isDepositing) {
            if (pad2.right_trigger_pressed()) {
                robot.verticalFlip.flipDown();
            } else {
                robot.verticalFlip.flipUp();
            }
        }

        // TOGGLE X to toggle back claw
        if (pad2.x_pressed) {
            robot.horizontalClaw.toggle();
        }

        // TOGGLE B to toggle front claw
        if (pad2.b_pressed) {
            robot.verticalClaw.toggle();
        }

        // PRESS A to retract
        if (pad2.a_pressed) {
            robot.retractArm();
        }

        if (pad2.y_pressed) {
            goingPos0 = false;
            goingPos1 = false;
            goingPos2 = false;
            goingPos3 = false;
            robot.verticalScheduler.clearGlobal();
            robot.verticalScheduler.clearLinear();
            robot.retractScheduler.clearGlobal();
            robot.retractScheduler.clearLinear();
            robot.isRetracting = false;
        }

        if (pad2.dpad_down_pressed) {
            multiTelemetry.addLine("Down pressed");
            robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.GROUND);
        }

        if (pad2.dpad_left_pressed) {
            multiTelemetry.addLine("Left pressed");
            robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.LOW);
        }

        if (pad2.dpad_up_pressed) {
            multiTelemetry.addLine("Up pressed");
            robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.MEDIUM);
        }

        if (pad2.dpad_right_pressed) {
            multiTelemetry.addLine("Right pressed");
            robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.HIGH);
        }

        if (pad2.right_bumper_pressed) {
            robot.depositCone();
        }

        // Hinge control, temporary
        if (pad2.left_bumper_pressed) {
            robot.hinge.setOffsetFactor(robot.hinge.getOffsetFactor() * -1);
        }
        robot.hinge.offsetPosition(pad2.gamepad.left_trigger);

        robot.verticalSlide.setPower(-pad2.get_partitioned_left_stick_y());
        if (!robot.isRetracting)
            robot.horizontalSlide.setPower(pad2.get_partitioned_left_stick_x());
        robot.moveLever(-pad2.get_partitioned_right_stick_y());
        robot.moveTurret(-pad2.get_partitioned_right_stick_x());


        fullTelemetry();
    }

    public void update() {
        pad1.update();
        pad2.update();
    }

    public void fullTelemetry() {
        multiTelemetry.addData("Horizontal motor encoder", robot.horizontalSlide.getPosition());
        multiTelemetry.addData("Vertical motor encoder", robot.verticalSlide.getPosition());
        multiTelemetry.addData("Limit switch:", robot.verticalSlide.getLimitState());
        multiTelemetry.addData("Vertical velocity", robot.verticalSlide.getVelocity());
        multiTelemetry.addData("Partitioned Left Y", pad2.get_partitioned_left_stick_y());
        multiTelemetry.addData("Partitioned Left X", pad2.get_partitioned_left_stick_x());
        multiTelemetry.addData("Partitioned Right Y", pad2.get_partitioned_right_stick_y());
        multiTelemetry.addData("Partitioned Right X", pad2.get_partitioned_right_stick_x());
        // multiTelemetry.addData("Vertical max power", robot.verticalSlide.maxPower);
        // multiTelemetry.addData("Vertical direction", robot.verticalSlide.getDirection());
        // multiTelemetry.addData("Vertical Powers", robot.verticalSlide.getPowers());
        multiTelemetry.addLine();
        multiTelemetry.addData("Vertical stopDir", robot.verticalSlide.stopDirection);
        multiTelemetry.addData("Horizontal stopDir", robot.horizontalSlide.stopDirection);
        multiTelemetry.addData("Retracting", robot.isRetracting);
        multiTelemetry.addData("Depositing", robot.isDepositing);
        // multiTelemetry.addData("Target", robot.horizontalSlide.motors[0].getTargetPosition());
        // multiTelemetry.addData("Vel", robot.horizontalSlide.motors[0].getVelocity());
        // multiTelemetry.addData("Power", robot.horizontalSlide.motors[0].getPower());
        // multiTelemetry.addData("Mode", robot.horizontalSlide.motors[0].getMode());
        multiTelemetry.addLine();
        multiTelemetry.addData("Turret pos", robot.turret.getPosition());
        multiTelemetry.addData("Lever pos", robot.lever.getPosition());
        multiTelemetry.addData("Hinge pos", robot.hinge.getPosition());
        multiTelemetry.update();
    }
}
