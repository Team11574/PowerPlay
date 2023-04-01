package incognito.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import incognito.cog.actions.Action;
import incognito.cog.actions.ActionManager;
import incognito.cog.hardware.gamepad.Cogtroller;
import incognito.cog.opmodes.RobotOpMode;
import incognito.teamcode.robot.WorldRobot;
import incognito.teamcode.robot.component.arm.HorizontalArm;
import incognito.teamcode.robot.component.arm.VerticalArm;

@TeleOp(name = "Front Arm Testing", group = "testing")
public class IdealTele extends RobotOpMode {
    // Instance variables
    WorldRobot robot;
    MultipleTelemetry multiTelemetry;
    Cogtroller pad1;
    Cogtroller pad2;
    Action intake;
    Action claw_out_of_way;

    @Override
    public void init() {
        this.robot = new WorldRobot(hardwareMap, telemetry, false);
        this.drivetrain = robot.drivetrain;
        this.multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pad1 = new Cogtroller(gamepad1);
        pad2 = new Cogtroller(gamepad2);

        multiTelemetry.addLine("Initialized!");
        multiTelemetry.update();

        initializeActions();
        initializeControls();
    }

    public void initializeActions() {
        intake = new Action(() -> {
                    robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE);
                    robot.horizontalArm.goToPosition(HorizontalArm.Position.WAIT_IN);
                })
                .until(() -> robot.verticalArm.atPosition() && robot.horizontalArm.atPosition())
                .then(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.IN))
                .until(robot.horizontalArm::atPosition)
                .delay(100)
                .then(robot.horizontalArm::openClaw);
        claw_out_of_way = new Action(
                () -> robot.horizontalArm.goToPosition(HorizontalArm.Position.WAIT_OUT))
                .until(robot.verticalArm.claw::isOpened)
                .then(() -> robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE));
    }

    public void initializeControls() {
        pad2.x.onRise(robot.verticalArm::toggleClaw);
        pad2.a.onRise(robot.horizontalArm::toggleClaw);
        pad2.right_trigger.onRise(robot.verticalArm::hingeDown);
        pad2.right_trigger.onFall(robot.verticalArm::hingeUp);
        pad2.dpad_down.onRise(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.GROUND));
        pad2.dpad_left.onRise(() -> robot.verticalArm.goToPosition(VerticalArm.Position.LOW))
                        .onRise(claw_out_of_way);
        pad2.dpad_up.onRise(() -> robot.verticalArm.goToPosition(VerticalArm.Position.MEDIUM))
                        .onRise(claw_out_of_way);
        pad2.dpad_right.onRise(() -> robot.verticalArm.goToPosition(VerticalArm.Position.HIGH))
                .onRise(claw_out_of_way);
        pad2.right_bumper.onRise(robot.horizontalArm::incrementLeverHeight);
        pad2.left_bumper.onRise(robot.horizontalArm::decrementLeverHeight);

    }

    @Override
    public void init_loop() {
        super.init_loop();
        fullTelemetry();
        update();
    }

    @Override
    public void loop() {
        fullTelemetry();
        update();
    }

    public void update() {
        robot.update();
        pad2.update();
        pad2.update();
        ActionManager.update();
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

    /*
    intake = new Action(() -> {
        robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE);
        robot.horizontalArm.goToPosition(HorizontalArm.Position.WAIT_IN);
    })
    .until(() -> robot.verticalArm.atPosition() && robot.horizontalArm.atPosition())
    .then(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.IN))
    .until(() -> robot.horizontalArm.atPosition())
    .delay(100)
    .then(() -> robot.horizontalArm.openClaw());


    */
}