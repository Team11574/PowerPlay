package incognito.teamcode.opmodes.tele;

import static incognito.teamcode.config.CameraConstants.JUNCTION_MAX_WIDTH;
import static incognito.teamcode.config.CameraConstants.JUNCTION_MIN_HORIZONTAL_DISTANCE;
import static incognito.teamcode.config.CameraConstants.JUNCTION_MIN_WIDTH;
import static incognito.teamcode.config.CameraConstants.JUNCTION_THETA_POWER_FACTOR;
import static incognito.teamcode.config.CameraConstants.JUNCTION_Y_POWER_FACTOR;
import static incognito.teamcode.config.GenericConstants.DRIVETRAIN_RAMP_SPEED;
import static incognito.teamcode.config.WorldSlideConstants.S_JOYSTICK_THRESHOLD;
import static incognito.teamcode.config.WorldSlideConstants.VS_CLAW_HANDOFF_SPEED;
import static incognito.teamcode.config.WorldSlideConstants.VS_DROP_SPEED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import incognito.cog.actions.Action;
import incognito.cog.actions.ActionManager;
import incognito.cog.hardware.gamepad.Cogtroller;
import incognito.cog.opmodes.RobotOpMode;
import incognito.cog.util.TelemetryBigError;
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
    Action conditional_intake;

    boolean targetLocking = false;

    double velX, velY, theta = 0;
    double normalFactor;
    double junctionWidth, junctionHorizontalDistance, junctionYPower;
    double frontRight_Power, backRight_Power, frontLeft_Power, backLeft_Power;
    double inputVelY, inputVelX, inputTheta;

    @Override
    public void init() {
        this.multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new WorldRobot(hardwareMap, multiTelemetry, false);
        this.drivetrain = robot.drivetrain;
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
                .delay(VS_DROP_SPEED)
                .then(() -> robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE));
        conditional_intake = new Action()
                .doIf(new Action(() -> intake.run())
                                .delay(VS_CLAW_HANDOFF_SPEED),
                    () -> !pad2.left_trigger_active() && robot.horizontalArm.getPosition() != HorizontalArm.Position.IN);
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
        pad2.b.onRise(() -> {
            telemetry.addLine("HERE 1");
            if (robot.verticalArm.getPosition() == VerticalArm.Position.INTAKE) {
                telemetry.addLine("HERE 2");
                robot.verticalArm.toggleClaw();
            } else {
                telemetry.addLine("HERE 3");
                robot.verticalArm.toggleClawWide();
            }
        });
        pad2.a.onRise(robot.horizontalArm::toggleClaw);
        pad2.right_trigger.onRise(robot.verticalArm::hingeDown);
        pad2.right_trigger.onFall(robot.verticalArm::hingeUp);
        pad2.dpad_down.onRise(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.GROUND));
        pad2.dpad_left.onRise(conditional_intake
                .then(() -> {
                    telemetry.addLine("LOWING");
                    robot.verticalArm.goToPosition(VerticalArm.Position.LOW);
                })
                .then(claw_out_of_way)
        );
        pad2.dpad_up.onRise(conditional_intake
                .then(() -> {
                    telemetry.addLine("MEDIUMING");
                    robot.verticalArm.goToPosition(VerticalArm.Position.MEDIUM);
                })
                .then(claw_out_of_way)
        );
        pad2.dpad_right.onRise(conditional_intake
                .then(() -> {
                    telemetry.addLine("HIGHING");
                    robot.verticalArm.goToPosition(VerticalArm.Position.HIGH);
                })
                .then(claw_out_of_way)
        );
        pad2.right_bumper.onRise(robot.horizontalArm::incrementLeverHeight);
        pad2.left_bumper.onRise(robot.horizontalArm::decrementLeverHeight);


        pad1.a.onRise(() -> {
            targetLocking = !targetLocking;
            if (targetLocking) {
                robot.autoCamera.startCamera();
            } else {
                robot.autoCamera.stopCamera();
            }
        });
    }

    @Override
    public void init_loop() {
        super.init_loop();
        fullTelemetry();
        update();
    }

    @Override
    public void loop() {
        // Adjust drivetrain
        if (!targetLocking) {
            adjustDrivetrain();
        } else {
            targetLock();
        }
        // If the left stick is moved (or was previously moved), set the slide by power. Resettable by calling .goToPosition()
        if (pad2.gamepad.left_stick_x > S_JOYSTICK_THRESHOLD || robot.horizontalArm.slide.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            robot.horizontalArm.setPower(pad2.gamepad.left_stick_x);
        }
        update();
        multiTelemetry.addData("Action count", ActionManager.actions.size());
        multiTelemetry.update();
        //fullTelemetry();
    }

    public void adjustDrivetrain() {
        inputVelY = -pad1.gamepad.left_stick_y;
        inputVelX = pad1.gamepad.left_stick_x;
        inputTheta = pad1.gamepad.right_stick_x;

        velY = ramp(inputVelY, velY, DRIVETRAIN_RAMP_SPEED, DRIVETRAIN_RAMP_SPEED * 3);
        velX = ramp(inputVelX, velX, DRIVETRAIN_RAMP_SPEED * 2, DRIVETRAIN_RAMP_SPEED * 3);
        theta = ramp(inputTheta, theta, 0.8);

        normalFactor = Math.max(Math.abs(velY) + Math.abs(velX) + Math.abs(theta), 1);
        frontRight_Power = (velY - velX - theta) / normalFactor;
        backRight_Power = (velY + velX - theta) / normalFactor;
        frontLeft_Power = (velY + velX + theta) / normalFactor;
        backLeft_Power = (velY - velX + theta) / normalFactor;

        drivetrain.setMotorPowers(frontLeft_Power, backLeft_Power, backRight_Power, frontRight_Power);
    }

    public void targetLock() {
        junctionWidth = robot.autoCamera.getJunctionWidth();
        junctionHorizontalDistance = robot.autoCamera.getJunctionDistance();
        // TODO: adjust JUNCTION_Y_POWER_FACTOR so the robot moves quickly when
        //  far away from the junction but slowly when close.
        if (junctionWidth == 0) {
            TelemetryBigError.raise(2);
            multiTelemetry.addLine("Junction locking failed");
            targetLocking = false;
            return;
        } else {
            junctionYPower = 1 / junctionWidth * JUNCTION_Y_POWER_FACTOR;
        }

        // TODO: Test, and consider removing. My thinking is that having the ability
        //  to move the robot in and out along the the autolock for precise movements
        //  could be good.
        velY = -pad1.get_partitioned_left_stick_y();

        // TODO: Test how close this is and adjust JUNCTION_WIDTH_THRESHOLD accordingly
        // TODO: See if we need different JUNCTION_MAX_WIDTH values for different
        //  junction heights, and if so create them.
        if (junctionWidth > JUNCTION_MAX_WIDTH
                || junctionWidth < JUNCTION_MIN_WIDTH) {
            // TODO: Consider adding in negative velocity if too close to the pole
            velY += 0;
        } else {
            velY += Math.max(0, junctionYPower);
        }
        velX = pad1.get_partitioned_left_stick_x();
        if (Math.abs(junctionHorizontalDistance) < JUNCTION_MIN_HORIZONTAL_DISTANCE) {
            theta = 0;
        } else {
            theta = junctionHorizontalDistance * JUNCTION_THETA_POWER_FACTOR;
        }

        multiTelemetry.addData("Junction distance", junctionHorizontalDistance);
        multiTelemetry.addData("Junction width", junctionWidth);
        multiTelemetry.addData("Junction Y power", velY);
        multiTelemetry.addData("Junction theta power", theta);


        normalFactor = Math.max(Math.abs(velY) + Math.abs(velX) + Math.abs(theta), 1);
        frontRight_Power = (velY - velX - theta) / normalFactor;
        backRight_Power = (velY + velX - theta) / normalFactor;
        frontLeft_Power = (velY + velX + theta) / normalFactor;
        backLeft_Power = (velY - velX + theta) / normalFactor;

        drivetrain.setMotorPowers(frontLeft_Power, backLeft_Power, backRight_Power, frontRight_Power);
    }

    private double ramp(double input, double currentValue, double speed) {
        return ramp(input, currentValue, speed, null);
    }

    private double ramp(Double input, Double currentValue, Double speed, Double speedDown) {
        double newValue = currentValue;
        if (currentValue > 0) {
            // Going forwards
            if (input > currentValue) {
                // Go forward faster
                newValue += Math.min(speed * (input - currentValue), input - currentValue);
            }
            else if (input < currentValue) {
                // Slow down
                if (speedDown == null)
                    newValue = input;
                else
                    // -0.1,  -1
                    newValue += Math.max(speedDown * (input - currentValue), input - currentValue);
            }
        }
        else {
            // Going backwards
            if (input < currentValue) {
                // Go backwards faster
                newValue += Math.max(speed * (input - currentValue), input - currentValue);
            }
            else if (input > currentValue) {
                // Slow down
                if (speedDown == null)
                    newValue = input;
                else
                    // currentValue = -1
                    // input = 0
                    // input - currentValue = 1
                    // ^ * speed = 0.1, 1
                    newValue += Math.min(speedDown * (input - currentValue), input - currentValue);
            }
        }

        if (Math.abs(currentValue) < speed && input == 0) {
            newValue = 0;
        }

        return newValue;
    }

    public void update() {
        robot.update();
        pad1.update();
        pad2.update();
        TelemetryBigError.update();
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
        multiTelemetry.addLine();
        multiTelemetry.addData("Horizontal hinge height", robot.horizontalArm.hinge.getPosition());
        multiTelemetry.addData("Lever height", robot.horizontalArm.leverOutPositionStorage);
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