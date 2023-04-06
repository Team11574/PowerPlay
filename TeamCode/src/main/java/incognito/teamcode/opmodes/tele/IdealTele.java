package incognito.teamcode.opmodes.tele;

import static incognito.teamcode.config.WorldSlideConstants.VS_CLAW_HANDOFF_SPEED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import incognito.cog.actions.Action;
import incognito.cog.actions.ActionManager;
import incognito.cog.hardware.gamepad.Cogtroller;
import incognito.cog.opmodes.RobotOpMode;
import incognito.teamcode.robot.WorldRobot;
import incognito.teamcode.robot.component.arm.HorizontalArm;
import incognito.teamcode.robot.component.arm.VerticalArm;

@TeleOp(name = "Ideal Tele", group = "testing")
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
        ActionManager.clear();

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
                //.delay(100)
                .then(robot.horizontalArm::openClaw);
        claw_out_of_way = new Action(
                () -> robot.horizontalArm.goToPosition(HorizontalArm.Position.CLAW_OUT))
                .until(robot.verticalArm.claw::isOpened)
                .delay(100)
                .then(() -> robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE));
    }

    public void initializeControls() {
        pad2.y.onRise(intake);
        pad2.x.onRise(() -> {
            if (robot.horizontalArm.getPosition() == HorizontalArm.Position.CLAW_OUT)
                robot.horizontalArm.goToPosition(HorizontalArm.Position.OUT);
            else if (robot.horizontalArm.getPosition() == HorizontalArm.Position.OUT)
                robot.horizontalArm.goToPosition(HorizontalArm.Position.MANUAL);
            else
                robot.horizontalArm.goToPosition(HorizontalArm.Position.CLAW_OUT);
        });
        pad2.b.onRise(robot.verticalArm::toggleClaw);
        pad2.a.onRise(robot.horizontalArm::toggleClaw);
        pad2.right_trigger.onRise(robot.verticalArm::hingeDown);
        pad2.right_trigger.onFall(robot.verticalArm::hingeUp);
        pad2.dpad_down.onRise(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.GROUND));
        pad2.dpad_left.onRise(new Action(() -> {
                    if (pad2.left_trigger_active()) {
                        intake.run();
                    }
                })
                .waitFor(intake)
                .delay(VS_CLAW_HANDOFF_SPEED)
                .then(() -> {
                    robot.verticalArm.goToPosition(VerticalArm.Position.LOW);
                    claw_out_of_way.run();
                }));
        pad2.dpad_left.onRise(new Action(() -> {
                    if (pad2.left_trigger_active()) {
                        intake.run();
                    }
                })
                .waitFor(intake)
                .delay(VS_CLAW_HANDOFF_SPEED)
                .then(() -> {
                    robot.verticalArm.goToPosition(VerticalArm.Position.MEDIUM);
                    claw_out_of_way.run();
                }));
        pad2.dpad_left.onRise(new Action(() -> {
                    if (pad2.left_trigger_active()) {
                        intake.run();
                    }
                })
                .waitFor(intake)
                .delay(VS_CLAW_HANDOFF_SPEED)
                .then(() -> {
                    robot.verticalArm.goToPosition(VerticalArm.Position.HIGH);
                    claw_out_of_way.run();
                }));
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
        update();
        fullTelemetry();
    }

    public void update() {
        robot.update();
        pad1.update();
        pad2.update();
        ActionManager.update();
    }

    public void fullTelemetry() {
        multiTelemetry.addData("Vertical claw isOpened", robot.verticalArm.claw.isOpened());
        multiTelemetry.addData("claw_out is active", claw_out_of_way.isActive());
        multiTelemetry.addData("claw_out index", claw_out_of_way.index);
        multiTelemetry.addData("intake is active", intake.isActive());
        multiTelemetry.addData("intake index", intake.index);
        multiTelemetry.addLine();
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
        multiTelemetry.addData("Horizontal distance", robot.horizontalArm.getDistance());
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