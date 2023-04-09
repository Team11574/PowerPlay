package incognito.teamcode.opmodes.tele;

import static incognito.teamcode.config.GenericConstants.JUNCTION_MAX_HORIZONTAL_DISTANCE;
import static incognito.teamcode.config.GenericConstants.JUNCTION_MIN_HORIZONTAL_DISTANCE;
import static incognito.teamcode.config.GenericConstants.DRIVETRAIN_RAMP_SPEED;
import static incognito.teamcode.config.GenericConstants.DRIVETRAIN_RIGHT_POWER_MULTIPLIER;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_AVERAGE;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_CONE_OFFSET;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_DISTANCE_FACTOR;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_DISTANCE_THRESHOLD;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_HIGH;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_LOW;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_MAX;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_MEDIUM;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_THETA_THRESHOLD;
import static incognito.teamcode.config.GenericConstants.JUNCTION_DISTANCE_PID;
import static incognito.teamcode.config.WorldSlideConstants.S_JOYSTICK_THRESHOLD;
import static incognito.teamcode.config.WorldSlideConstants.VS_CLAW_GRAB_SPEED;
import static incognito.teamcode.config.WorldSlideConstants.VS_CLAW_HANDOFF_SPEED;
import static incognito.teamcode.config.WorldSlideConstants.VS_CLAW_DROP_SPEED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import incognito.cog.actions.Action;
import incognito.cog.actions.ActionManager;
import incognito.cog.hardware.gamepad.Cogtroller;
import incognito.cog.opmodes.RobotOpMode;
import incognito.cog.util.PIDController;
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
    Action intake_delay;
    Action conditional_intake;

    boolean targetLocking = false;
    boolean fieldOriented = false;
    PIDController junctionDistancePID = new PIDController(JUNCTION_DISTANCE_PID);

    double velX, velY, theta, rotX, rotY, botHeading = 0;
    double normalFactor;
    double junctionMinDistance, junctionHorizontalDistance;
    double frontRightPower, backRightPower, frontLeftPower, backLeftPower;
    double inputVelY, inputVelX, inputTheta;

    @Override
    public void init() {
        this.multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new WorldRobot(hardwareMap, multiTelemetry, true);
        TelemetryBigError.initialize(multiTelemetry);
        robot.autoCamera.swapDoingJunctions();
        this.drivetrain = robot.drivetrain;
        pad1 = new Cogtroller(gamepad1);
        pad2 = new Cogtroller(gamepad2);

        ActionManager.clear();

        initializeActions();
        initializeControls();
    }

    public void initializeActions() {
        intake = new Action(() -> {
                    robot.verticalArm.openClaw();
                    if (robot.horizontalArm.getPosition() != HorizontalArm.Position.IN)
                        robot.horizontalArm.goToPosition(HorizontalArm.Position.WAIT_IN);
                })
                .until(() -> robot.verticalArm.atPosition() && robot.horizontalArm.atPosition())
                .then(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.IN))
                .until(robot.horizontalArm::atPosition)
                //.delay(100)
                .then(robot.horizontalArm::openClaw);
        claw_out_of_way = new Action()
                //.until(robot.horizontalArm::atPosition)
                .then(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.CLAW_OUT))
                .until(robot.verticalArm.claw::isOpened)
                .delay(VS_CLAW_DROP_SPEED)
                .then(() -> robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE));
        intake_delay = intake
                .delay(VS_CLAW_HANDOFF_SPEED);
        conditional_intake = new Action()
                .doIf(intake_delay,
                    () -> !pad2.left_trigger_active() && robot.horizontalArm.getPosition() != HorizontalArm.Position.IN)
                .then(robot.verticalArm::closeClaw)
                .then(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.WAIT_OUT))
                .delay(VS_CLAW_GRAB_SPEED);
    }

    public void initializeControls() {
        pad2.b.onRise(intake);
        pad2.a.onRise(() -> {
            if (robot.horizontalArm.getPosition() == HorizontalArm.Position.CLAW_OUT
                    || robot.horizontalArm.getPosition() == HorizontalArm.Position.MANUAL) {
                if (pad2.left_trigger_active()) {
                    robot.horizontalArm.goToPosition(HorizontalArm.Position.SUPER_OUT);
                } else {
                    robot.verticalArm.openClaw();
                    robot.horizontalArm.goToPosition(HorizontalArm.Position.OUT);
                }
            } else if (robot.horizontalArm.getPosition() == HorizontalArm.Position.OUT) {
                robot.horizontalArm.goToPosition(HorizontalArm.Position.MANUAL);
            } else {
                robot.horizontalArm.goToPosition(HorizontalArm.Position.CLAW_OUT);
            }
        });
        pad2.y.onRise(() -> {
            if (robot.verticalArm.getPosition() == VerticalArm.Position.INTAKE) {
                robot.verticalArm.toggleClaw();
            } else {
                robot.verticalArm.toggleClawWide();
            }
        });
        pad2.x.onRise(robot.horizontalArm::toggleClaw);
        pad2.right_trigger.onRise(robot.verticalArm::hingeDown);
        pad2.right_trigger.onFall(robot.verticalArm::hingeUp);
        pad2.dpad_down.onRise(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.GROUND));
        pad2.dpad_left.onRise(conditional_intake
                .then(() -> robot.verticalArm.goToPosition(VerticalArm.Position.LOW))
                .then(claw_out_of_way)
        );
        pad2.dpad_up.onRise(conditional_intake
                .then(() -> robot.verticalArm.goToPosition(VerticalArm.Position.MEDIUM))
                .then(claw_out_of_way)
        );
        pad2.dpad_right.onRise(conditional_intake
                .then(() -> robot.verticalArm.goToPosition(VerticalArm.Position.HIGH))
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
        Action oriented = new Action().doIf(this::swapFieldOriented, () -> !targetLocking);
        pad1.y.onRise(oriented);
    }

    @Override
    public void init_loop() {
        super.init_loop();
        //fullTelemetry();
        update();
    }

    @Override
    public void loop() {
        // Adjust drivetrain
        if (!targetLocking) {
            fieldOriented = false;
            adjustVelocities();
        } else {
            targetLock();
        }
        // If the left stick is moved (or was previously moved), set the slide by power. Resettable by calling .goToPosition()
        if (pad2.gamepad.left_stick_x > S_JOYSTICK_THRESHOLD || robot.horizontalArm.slide.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            robot.horizontalArm.setPower(pad2.gamepad.left_stick_x);
        }

        updatePower();

        update();
        /*multiTelemetry.addData("Target Locking", targetLocking);
        multiTelemetry.addData("Field Oriented Movement", fieldOriented);
        multiTelemetry.addLine();
        multiTelemetry.addData("Front distance", getFrontDistance());
        multiTelemetry.addData("Horizontal distance", robot.horizontalArm.getDistance());
        multiTelemetry.addData("Vertical hinge pos", robot.verticalArm.hinge.getPosition());
        multiTelemetry.addData("Vertical claw pos", robot.verticalArm.claw.getPosition());
        multiTelemetry.update();*/
        //fullTelemetry();
    }

    public void swapFieldOriented() {
        fieldOriented = !fieldOriented;
    }

    public void updatePower() {
        if (fieldOriented && !targetLocking) {
            botHeading = drivetrain.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            rotX = inputVelX * Math.cos(-botHeading) - inputVelY * Math.sin(-botHeading);
            rotY = inputVelX * Math.sin(-botHeading) + inputVelY * Math.cos(-botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            normalFactor = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(theta), 1);
            frontLeftPower = (rotY + rotX + inputTheta) / normalFactor;
            backLeftPower = (rotY - rotX + inputTheta) / normalFactor;
            frontRightPower = (rotY - rotX - inputTheta) / normalFactor;
            backRightPower = (rotY + rotX - inputTheta) / normalFactor;
        } else {
            normalFactor = Math.max(Math.abs(velY) + Math.abs(velX) + Math.abs(theta), 1);
            frontRightPower = (velY - velX - theta) / normalFactor * DRIVETRAIN_RIGHT_POWER_MULTIPLIER;
            backRightPower = (velY + velX - theta) / normalFactor * DRIVETRAIN_RIGHT_POWER_MULTIPLIER;
            frontLeftPower = (velY + velX + theta) / normalFactor;
            backLeftPower = (velY - velX + theta) / normalFactor;
        }
        drivetrain.setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);
    }

    public void adjustVelocities() {
        inputVelY = -pad1.gamepad.left_stick_y;
        inputVelX = pad1.gamepad.left_stick_x;
        inputTheta = pad1.gamepad.right_stick_x;

        velY = ramp(inputVelY, velY, DRIVETRAIN_RAMP_SPEED, DRIVETRAIN_RAMP_SPEED * 3);
        velX = ramp(inputVelX, velX, DRIVETRAIN_RAMP_SPEED * 2, DRIVETRAIN_RAMP_SPEED * 3);
        theta = ramp(inputTheta, theta, 0.6);
    }

    public void targetLock() {
        junctionHorizontalDistance = robot.autoCamera.getJunctionDistance();
        velX = pad1.gamepad.left_stick_x;
        velY = -pad1.gamepad.left_stick_y;
        junctionDistancePID.setDesiredValue(0);
        if (Math.abs(junctionHorizontalDistance) < JUNCTION_MIN_HORIZONTAL_DISTANCE
                || Math.abs(junctionHorizontalDistance) > JUNCTION_MAX_HORIZONTAL_DISTANCE) {
            theta = 0;
        } else {
            theta = junctionDistancePID.update(junctionHorizontalDistance); // junctionHorizontalDistance * JUNCTION_THETA_POWER_FACTOR;
            // Only move forward once we are locked on horizontally to the junction if using REV sensor
            junctionMinDistance = getPreferredJunctionDistance();
            if (theta < FRONT_DS_THETA_THRESHOLD) {
                if (Math.abs(getFrontDistance() - junctionMinDistance) < FRONT_DS_DISTANCE_THRESHOLD
                        || getFrontDistance() > FRONT_DS_MAX) {
                    velY += 0;
                } else {
                    // Add an amount of power proportional to the distance from the junction
                    //  to the robot
                    velY += (getFrontDistance() - junctionMinDistance) / (FRONT_DS_DISTANCE_FACTOR);
                }
            }
        }

        multiTelemetry.addData("Junction horizontal distance", junctionHorizontalDistance);
        multiTelemetry.addData("Junction forward pref distance", junctionMinDistance);
        multiTelemetry.addData("Junction forward actual distance", getFrontDistance());
        multiTelemetry.addData("Junction Y power", velY);
        multiTelemetry.addData("Junction theta power", theta);
        multiTelemetry.addLine();
    }

    public double  getFrontDistance() {
        // In CM
        return robot.frontDistanceSensor.getDistance(DistanceUnit.CM);
    }

    public double getPreferredJunctionDistance() {
        double distance;
        switch (robot.verticalArm.getPosition()) {
            case LOW: distance = FRONT_DS_LOW; break;
            case MEDIUM: distance = FRONT_DS_MEDIUM; break;
            case HIGH: distance = FRONT_DS_HIGH; break;
            default: distance = FRONT_DS_AVERAGE; break;
        }

        if (!robot.autoCamera.coneOnJunction()) {
            distance += FRONT_DS_CONE_OFFSET;
        }
        return distance;
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