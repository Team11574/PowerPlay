package incognito.teamcode.opmodes.testing;

import static incognito.teamcode.config.WorldSlideConstants.HS_HINGE_END;
import static incognito.teamcode.config.WorldSlideConstants.HS_HINGE_START;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        if (pad2.a_pressed) {
            robot.horizontalArm.openClaw();
        }
        if (pad2.b_pressed) {
            robot.horizontalArm.closeClaw();
        }

        if (pad2.y_pressed) {
            robot.horizontalArm.goToPosition(HorizontalArm.Position.IN);
        }
        if (pad2.x_pressed) {
            robot.horizontalArm.goToPosition(HorizontalArm.Position.OUT);
        }
        if (pad2.right_bumper_pressed) {
            robot.horizontalArm.goToPosition(HorizontalArm.Position.MANUAL);
        }

        if (pad2.dpad_right_pressed) {
            robot.horizontalArm.hinge.setPosition(HS_HINGE_START);
        }
        if (pad2.dpad_left_pressed) {
            robot.horizontalArm.hinge.setPosition(HS_HINGE_END);
        }



        if (pad2.dpad_up_pressed) {
            robot.horizontalArm.incrementLeverHeight();
        }

        if (pad2.dpad_down_pressed) {
            robot.horizontalArm.decrementLeverHeight();
        }

        multiTelemetry.addData("Lever stored", robot.horizontalArm.leverOutPositionStorage);
        multiTelemetry.addData("lever increment", robot.horizontalArm.leverOutPositionStorage.increment());
        multiTelemetry.addData("lever increment", robot.horizontalArm.leverOutPositionStorage.decrement());
        fullTelemetry();
        robot.update();
        pad2.update();
    }

    public void fullTelemetry() {
        multiTelemetry.addData("Vertical slide motor encoder", robot.horizontalArm.slide.getPosition());
        multiTelemetry.addData("Vertical slide target position", robot.horizontalArm.slide.getTargetPosition());
        multiTelemetry.addData("Vertical slide motor power", robot.horizontalArm.slide.getPower());
        multiTelemetry.addData("Vertical slide limit switch:", robot.horizontalArm.slide.getDangerState());
        multiTelemetry.addLine();
        multiTelemetry.addData("Vertical slide atTop", robot.horizontalArm.slide.atTop);
        multiTelemetry.addData("Vertical slide atBottom", robot.horizontalArm.slide.atBottom);
        multiTelemetry.addData("Vertical slide goingDown", robot.horizontalArm.slide.goingDown());
        multiTelemetry.addData("Vertical slide goingUp", robot.horizontalArm.slide.goingUp());
        multiTelemetry.addLine();
        multiTelemetry.addData("Vertical lever pos", robot.horizontalArm.lever.getPosition());
        multiTelemetry.addData("Vertical hinge pos", robot.horizontalArm.hinge.getPosition());
        multiTelemetry.addData("Vertical arm pos", robot.horizontalArm.getPosition());
        multiTelemetry.addLine();
        multiTelemetry.update();
    }
}