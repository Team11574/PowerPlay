package incognito.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import incognito.cog.actions.Scheduler;
import incognito.cog.hardware.gamepad.GamepadPlus;
import incognito.cog.opmodes.RobotOpMode;
import incognito.teamcode.robot.WorldRobot;
import incognito.teamcode.robot.component.arm.HorizontalArm;
import incognito.teamcode.robot.component.arm.VerticalArm;

@TeleOp(name = "Front Arm Testing", group = "testing")
public class FrontArmTesting extends RobotOpMode {
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
        if (pad2.x_pressed) {
            robot.verticalArm.toggleClaw();
        }
        if (pad2.a_pressed) {
            robot.horizontalArm.toggleClaw();
        }

        if (pad2.right_trigger_pressed) {
            robot.verticalArm.hingeDown();
        }
        if (pad2.right_trigger_depressed) {
            robot.verticalArm.hingeUp();
        }
        
        if (robot.verticalArm.getLastPosition() != VerticalArm.Position.INTAKE
            && robot.verticalArm.getPosition() == VerticalArm.Position.INTAKE) {
            robot.horizontalArm.goToPosition(HorizontalArm.Position.HOLD_CONE);
        }
        if (robot.verticalArm.getLastPosition() == VerticalArm.Position.INTAKE
            && robot.verticalArm.getPosition() != VerticalArm.Position.INTAKE) {
            robot.horizontalArm.goToPosition(HorizontalArm.Position.UP);
        }

        if (pad2.dpad_down_pressed) {
            robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE);
        }
        if (robot.verticalArm.atPosition()) {
            if (robot.horizontalArm.getPosition() == HorizontalArm.Position.HOLD_CONE) {
                //if (robot.verticalArm.lever.atSetPosition(900)); {
                robot.horizontalArm.goToPosition(HorizontalArm.Position.IN);
            }
        }
        if (robot.horizontalArm.atPosition()) {
            if (robot.horizontalArm.getPosition() == HorizontalArm.Position.UP) {
                robot.horizontalArm.goToPosition(HorizontalArm.Position.CLAW_OUT);
            } else if (robot.horizontalArm.getPosition() == HorizontalArm.Position.HOLD_CONE) {
                robot.horizontalArm.openClaw();
            }
        }
        if (pad2.dpad_left_pressed) {
            robot.verticalArm.goToPosition(VerticalArm.Position.LOW);
        }
        if (pad2.dpad_up_pressed) {
            robot.verticalArm.goToPosition(VerticalArm.Position.MEDIUM);

        }
        if (pad2.dpad_right_pressed) {
            robot.verticalArm.goToPosition(VerticalArm.Position.HIGH);
        }

        fullTelemetry();
        robot.update();
        pad2.update();
    }

    public void fullTelemetry() {
        multiTelemetry.addData("Vertical slide motor encoder", robot.verticalArm.slide.getPosition());
        multiTelemetry.addData("Vertical slide target position", robot.verticalArm.slide.getTargetPosition());
        multiTelemetry.addData("Vertical slide motor power", robot.verticalArm.slide.getPower());
        multiTelemetry.addData("Vertical slide limit switch:", robot.verticalArm.slide.getDangerState());
        multiTelemetry.addLine();
        multiTelemetry.addData("Vertical slide atTop", robot.verticalArm.slide.atTop);
        multiTelemetry.addData("Vertical slide atBottom", robot.verticalArm.slide.atBottom);
        multiTelemetry.addData("Vertical slide goingDown", robot.verticalArm.slide.goingDown());
        multiTelemetry.addData("Vertical slide goingUp", robot.verticalArm.slide.goingUp());
        multiTelemetry.addLine();
        multiTelemetry.addData("Vertical lever pos", robot.verticalArm.lever.getPosition());
        multiTelemetry.addData("Vertical hinge pos", robot.verticalArm.hinge.getPosition());
        multiTelemetry.addData("Vertical arm pos", robot.verticalArm.getPosition());
        multiTelemetry.addLine();
        multiTelemetry.addData("Vertical arm atPosition", robot.verticalArm.atPosition());
        multiTelemetry.addData("Horizontal arm atPosition", robot.horizontalArm.atPosition());
        multiTelemetry.update();
    }
}