package incognito.teamcode.opmodes.testing;

import static incognito.teamcode.config.WorldSlideConstants.HS_HINGE_OUT;
import static incognito.teamcode.config.WorldSlideConstants.HS_HINGE_IN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import incognito.cog.actions.Scheduler;
import incognito.cog.hardware.gamepad.GamepadPlus;
import incognito.cog.opmodes.RobotOpMode;
import incognito.teamcode.robot.WorldRobot;
import incognito.teamcode.robot.component.arm.HorizontalArm;
import incognito.teamcode.robot.component.arm.VerticalArm;
import incognito.teamcode.robot.component.servoImplementations.Lever;

@TeleOp(name = "Back Arm Testing", group = "testing")
public class BackArmTesting extends RobotOpMode {
    // Instance variables
    WorldRobot robot;
    MultipleTelemetry multiTelemetry;
    Scheduler scheduler;
    GamepadPlus pad2;

    @Override
    public void init() {

        this.robot = new WorldRobot(hardwareMap, telemetry, false);
        this.drivetrain = robot.drivetrain;
        this.multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.scheduler = new Scheduler();
        pad2 = new GamepadPlus(gamepad2);

        multiTelemetry.addLine("Yippee!");
        multiTelemetry.update();

    }

    @Override
    public void init_loop() {
        super.init_loop();
        robot.update();
        fullTelemetry();
    }

    @Override
    public void loop() {
        /*if (pad2.a_pressed) {
            robot.horizontalArm.openClaw();
        }
        if (pad2.b_pressed) {
            robot.horizontalArm.closeClaw();
        }*/

        if (pad2.y_pressed) {
            robot.horizontalArm.goToPosition(HorizontalArm.Position.IN);
        }
        if (pad2.x_pressed) {
            robot.horizontalArm.goToPosition(HorizontalArm.Position.CLAW_OUT);
        }

        if (pad2.dpad_right_pressed) {
            robot.horizontalArm.hinge.setPosition(HS_HINGE_IN);
        }
        if (pad2.dpad_left_pressed) {
            robot.horizontalArm.hinge.setPosition(HS_HINGE_OUT);
        }

        if (pad2.a_pressed) {
            if (pad2.left_trigger_active()) {
                robot.horizontalArm.hinge.offsetPosition(0.02);
            } else {
                robot.horizontalArm.hinge.offsetPosition(0.005);
            }
        }
        if (pad2.b_pressed) {
            if (pad2.left_trigger_active()) {
                robot.horizontalArm.hinge.offsetPosition(-0.02);
            } else {
                robot.horizontalArm.hinge.offsetPosition(-0.005);
            }
        }



        if (pad2.dpad_up_pressed) {
            robot.horizontalArm.decrementLeverHeight();
            robot.horizontalArm.levelHinge();
        }

        if (pad2.dpad_down_pressed) {
            robot.horizontalArm.incrementLeverHeight();
            robot.horizontalArm.levelHinge();
        }

        multiTelemetry.addData("Lever stored", robot.horizontalArm.leverOutPositionStorage);
        multiTelemetry.addData("lever increment", robot.horizontalArm.leverOutPositionStorage.increment());
        multiTelemetry.addData("lever increment", robot.horizontalArm.leverOutPositionStorage.decrement());
        fullTelemetry();
        robot.update();
        pad2.update();
    }

    public void fullTelemetry() {
        multiTelemetry.addData("Horizontal slide motor encoder", robot.horizontalArm.slide.getPosition());
        multiTelemetry.addData("Horizontal slide target position", robot.horizontalArm.slide.getTargetPosition());
        multiTelemetry.addData("Horizontal slide motor power", robot.horizontalArm.slide.getPower());
        multiTelemetry.addData("Horizontal slide limit switch:", robot.horizontalArm.slide.getDangerState());
        multiTelemetry.addLine();
        multiTelemetry.addData("Horizontal slide atTop", robot.horizontalArm.slide.atTop);
        multiTelemetry.addData("Horizontal slide atBottom", robot.horizontalArm.slide.atBottom);
        multiTelemetry.addData("Horizontal slide goingDown", robot.horizontalArm.slide.goingDown());
        multiTelemetry.addData("Horizontal slide goingUp", robot.horizontalArm.slide.goingUp());
        multiTelemetry.addLine();
        multiTelemetry.addData("Horizontal lever pos", robot.horizontalArm.lever.getPosition());
        multiTelemetry.addData("Horizontal hinge pos", robot.horizontalArm.hinge.getPosition());
        multiTelemetry.addData("Horizontal claw pos", robot.horizontalArm.claw.getPosition());
        multiTelemetry.addData("Horizontal arm pos", robot.horizontalArm.getPosition());
        multiTelemetry.addLine();
        multiTelemetry.addData("Horizontal distance", robot.horizontalArm.getDistance());
        multiTelemetry.update();
    }
}