package incognito.teamcode.opmodes.tele;

import static incognito.teamcode.config.GenericConstants.DRIVETRAIN_LEFT_POWER_MULTIPLIER;
import static incognito.teamcode.config.GenericConstants.DRIVETRAIN_SLOW_MODE_POWER;
import static incognito.teamcode.config.GenericConstants.FORWARD_DISTANCE_PID;
import static incognito.teamcode.config.GenericConstants.JUNCTION_MAX_HORIZONTAL_DISTANCE;
import static incognito.teamcode.config.GenericConstants.JUNCTION_MIN_HORIZONTAL_DISTANCE;
import static incognito.teamcode.config.GenericConstants.DRIVETRAIN_RAMP_SPEED;
import static incognito.teamcode.config.GenericConstants.DRIVETRAIN_RIGHT_POWER_MULTIPLIER;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_AVERAGE;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_CONE_OFFSET;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_DISTANCE_THRESHOLD;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_HIGH;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_LOW;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_MAX;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_MEDIUM;
import static incognito.teamcode.config.GenericConstants.JUNCTION_DISTANCE_PID;
import static incognito.teamcode.config.GenericConstants.JUNCTION_PREFERRED_PIXEL_DISTANCE_CLOSE;
import static incognito.teamcode.config.GenericConstants.JUNCTION_PREFERRED_PIXEL_DISTANCE_FAR;
import static incognito.teamcode.config.GenericConstants.JUNCTION_THETA_DISTANCE_FACTOR;
import static incognito.teamcode.config.GenericConstants.JUNCTION_WIDTH_TO_DISTANCE_FACTOR;
import static incognito.teamcode.config.GenericConstants.MAX_THETA_POWER;
import static incognito.teamcode.config.GenericConstants.MAX_Y_POWER;
import static incognito.teamcode.config.WorldSlideConstants.HS_CLAW_DROP_SPEED;
import static incognito.teamcode.config.WorldSlideConstants.HS_CLAW_GRAB_SPEED;
import static incognito.teamcode.config.WorldSlideConstants.S_JOYSTICK_THRESHOLD;
import static incognito.teamcode.config.WorldSlideConstants.VS_CLAW_GRAB_SPEED;
import static incognito.teamcode.config.WorldSlideConstants.VS_CLAW_HANDOFF_SPEED;
import static incognito.teamcode.config.WorldSlideConstants.VS_CLAW_DROP_SPEED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
import incognito.teamcode.robot.component.servoImplementations.HorizontalHinge;

@TeleOp(name = "Ideal Tele", group = "0tele")
public class IdealTele extends RobotOpMode {
    // Instance variables
    WorldRobot robot;
    MultipleTelemetry multiTelemetry;
    Cogtroller pad1;
    Cogtroller pad2;
    Action intake;
    Action vertical_arm_to_intake;
    Action intake_for_cycle;
    Action conditional_intake;
    Action super_intake;
    Action conditional_ground;
    Action horizontal_hinge_down;
    Action horizontal_hinge_up;

    boolean targetLocking = false;
    PIDController junctionDistancePID = new PIDController(JUNCTION_DISTANCE_PID);
    PIDController forwardDistancePID = new PIDController(FORWARD_DISTANCE_PID);

    double velX, velY, theta = 0;
    double normalFactor;
    double junctionMinDistance, junctionHorizontalDistance;
    double frontRightPower, backRightPower, frontLeftPower, backLeftPower;
    double inputVelY, inputVelX, inputTheta;

    double drivetrainPowerFactor = 1;

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
        /*
        press B to intake will end with back claw UP
        if then press Junction height and you know you have intaken already, go to the height without adjusting back claw at all
        if height is pressed to intake, then have back claw go back out normally
        super B can still do super intake

         */
        intake = new Action(robot.horizontalArm::closeClaw)
                .delay(HS_CLAW_GRAB_SPEED)
                .then(() -> {
                    robot.verticalArm.openClaw();
                    if (robot.verticalArm.getPosition() != VerticalArm.Position.INTAKE) {
                        robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE);
                        robot.horizontalArm.goToPosition(HorizontalArm.Position.UP_CLOSED);
                    } else if (robot.horizontalArm.getPosition() != HorizontalArm.Position.IN) {
                        robot.horizontalArm.goToPosition(HorizontalArm.Position.WAIT_IN);
                    }
                })
                .until(() -> robot.verticalArm.atPosition() && robot.horizontalArm.atPosition())
                .then(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.IN))
                .until(robot.horizontalArm::atPosition)
                .then(robot.horizontalArm::openClaw)
                .delay(HS_CLAW_DROP_SPEED)
                .then(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.UP))
                .delay(VS_CLAW_HANDOFF_SPEED)
                .then(robot.verticalArm::closeClaw)
                .globalize();
        vertical_arm_to_intake = new Action()
                .until(robot.verticalArm.claw::isOpened)
                .delay(VS_CLAW_DROP_SPEED)
                .then(() -> robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE))
                .doIf(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.UP),
                        () -> robot.horizontalArm.getPosition() == HorizontalArm.Position.IN)
                .globalize();
        intake_for_cycle = intake
                .then(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.CLAW_OUT))
                .delay(VS_CLAW_GRAB_SPEED)
                .globalize();
        super_intake = new Action()
                .doIf(() -> {
                    robot.horizontalArm.goToPosition(HorizontalArm.Position.IN);
                    robot.horizontalArm.openClaw();
                    },
                        () -> pad2.left_trigger_active())
                .doIf(robot.verticalArm::openClaw,
                        () -> !pad2.left_trigger_active() && robot.horizontalArm.getPosition() == HorizontalArm.Position.UP)
                .doIf(intake,
                    () -> !pad2.left_trigger_active() && robot.horizontalArm.getPosition() != HorizontalArm.Position.UP)
                .globalize();
        conditional_intake = new Action()
                .doIf(intake_for_cycle,
                    () -> robot.horizontalArm.getPosition() != HorizontalArm.Position.UP)
                .then(robot.verticalArm::closeClaw)
                .globalize();
        conditional_ground = new Action()
                .doIf(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.UP_CLOSED),
                        () -> robot.horizontalArm.getPosition() != HorizontalArm.Position.UP_GROUND)
                .doIf(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.GROUND),
                        () -> robot.horizontalArm.getPosition() == HorizontalArm.Position.UP_GROUND)
                .doIf(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.UP_GROUND),
                        () -> robot.horizontalArm.getPosition() == HorizontalArm.Position.UP_CLOSED)
                .globalize();
        horizontal_hinge_down = new Action()
                .doIf(() -> robot.horizontalArm.hinge.goToSetPosition(HorizontalHinge.Position.FIFTH),
                    () -> robot.horizontalArm.getPosition() == HorizontalArm.Position.UP_GROUND)
                .globalize();
        horizontal_hinge_up = new Action()
                .doIf(() -> robot.horizontalArm.hinge.goToSetPosition(HorizontalHinge.Position.MID),
                        () -> robot.horizontalArm.getPosition() == HorizontalArm.Position.UP_GROUND)
                .globalize();
    }

    public void initializeControls() {
        pad2.b.onRise(super_intake);
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
        pad2.right_trigger.onRise(horizontal_hinge_down);
        pad2.right_trigger.onFall(robot.verticalArm::hingeUp);
        pad2.right_trigger.onFall(horizontal_hinge_up);
        pad2.dpad_down.onRise(conditional_ground);
        pad2.dpad_left.onRise(conditional_intake
                .then(() -> robot.verticalArm.goToPosition(VerticalArm.Position.LOW))
                .then(vertical_arm_to_intake)
        );
        pad2.dpad_up.onRise(conditional_intake
                .then(() -> robot.verticalArm.goToPosition(VerticalArm.Position.MEDIUM))
                .then(vertical_arm_to_intake)
        );
        pad2.dpad_right.onRise(conditional_intake
                .then(() -> robot.verticalArm.goToPosition(VerticalArm.Position.HIGH))
                .then(vertical_arm_to_intake)
        );
        pad2.left_bumper.onRise(robot.horizontalArm::incrementLeverHeight);
        pad2.right_bumper.onRise(robot.horizontalArm::decrementLeverHeight);
        pad2.guide.onRise(() -> {
            intake.cancel();
            vertical_arm_to_intake.cancel();
            conditional_intake.cancel();
            intake_for_cycle.cancel();
            targetLocking = false;
            robot.horizontalArm.slide.enable();
            robot.verticalArm.slide.enable();
            robot.horizontalArm.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.horizontalArm.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        });



        pad1.right_trigger.onRise(() -> {
            targetLocking = true;
            junctionDistancePID.reset();
            forwardDistancePID.reset();
            robot.autoCamera.startCamera();
        });
        pad1.right_trigger.onFall(() -> {
            targetLocking = false;
            robot.autoCamera.stopCamera();
            velX = 0;
            velY = 0;
            theta = 0;
        });
        pad1.left_trigger.onRise(() -> drivetrainPowerFactor = DRIVETRAIN_SLOW_MODE_POWER);
        pad1.left_trigger.onFall(() -> drivetrainPowerFactor = 1);
        /*Action oriented = new Action().doIf(this::swapFieldOriented, () -> !targetLocking);
        pad1.y.onRise(oriented);*/
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
        if (targetLocking) {
            targetLock();
        } else {
            adjustVelocities();
        }
        // If the left stick is moved (or was previously moved), set the slide by power. Resettable by calling .goToPosition()
        if (pad2.gamepad.left_stick_x > S_JOYSTICK_THRESHOLD || robot.horizontalArm.slide.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            robot.horizontalArm.setPower(pad2.gamepad.left_stick_x);
        }
        if (Math.abs(pad2.gamepad.right_stick_y) > S_JOYSTICK_THRESHOLD && robot.verticalArm.getPosition() != VerticalArm.Position.INTAKE) {
            robot.verticalArm.setPower(-pad2.gamepad.right_stick_y);
        }

        multiTelemetry.update();
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

    public void updatePower() {
        normalFactor = Math.max(Math.abs(velY) + Math.abs(velX) + Math.abs(theta), 1);
        frontRightPower = (velY - velX - theta) / normalFactor * DRIVETRAIN_RIGHT_POWER_MULTIPLIER * drivetrainPowerFactor;
        backRightPower = (velY + velX - theta) / normalFactor * DRIVETRAIN_RIGHT_POWER_MULTIPLIER * drivetrainPowerFactor;
        frontLeftPower = (velY + velX + theta) / normalFactor * DRIVETRAIN_LEFT_POWER_MULTIPLIER * drivetrainPowerFactor;
        backLeftPower = (velY - velX + theta) / normalFactor * DRIVETRAIN_LEFT_POWER_MULTIPLIER * drivetrainPowerFactor;
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

        if (robot.autoCamera.coneOnJunction()) {
            distance += FRONT_DS_CONE_OFFSET;
        }
        return distance;
    }

    public double getPreferredJunctionPixelDistance() {
        if (getFrontDistance() > FRONT_DS_MAX) {
            return JUNCTION_PREFERRED_PIXEL_DISTANCE_FAR;
        } else {
            return JUNCTION_PREFERRED_PIXEL_DISTANCE_CLOSE;
        }
    }

    public void targetLock() {
        junctionHorizontalDistance = robot.autoCamera.getJunctionDistance();
        velX = pad1.gamepad.left_stick_x;
        velY = -pad1.gamepad.left_stick_y;
        theta = pad1.gamepad.right_stick_x;
        double junctionPreferredPixelDistance = getPreferredJunctionPixelDistance();
        junctionDistancePID.setDesiredValue(junctionPreferredPixelDistance);

        // Set preferred front distance
        double junctionPreferredFrontDistance = getPreferredJunctionDistance();
        double junctionWidthDistance = JUNCTION_WIDTH_TO_DISTANCE_FACTOR / robot.autoCamera.getJunctionWidth();
        forwardDistancePID.setDesiredValue(junctionPreferredFrontDistance);
        double distanceUsed = getFrontDistance();
        if (getFrontDistance() > FRONT_DS_MAX) {
            distanceUsed = junctionWidthDistance;
        }

        if (Math.abs(junctionHorizontalDistance - junctionPreferredPixelDistance) < JUNCTION_MIN_HORIZONTAL_DISTANCE
                || Math.abs(junctionHorizontalDistance - junctionPreferredPixelDistance) > JUNCTION_MAX_HORIZONTAL_DISTANCE) {
            theta += 0;
        } else {
            theta += Math.min(MAX_THETA_POWER, junctionDistancePID.update(junctionHorizontalDistance) / (distanceUsed * JUNCTION_THETA_DISTANCE_FACTOR));
        }
        if (distanceUsed < 0) {
            velY += 0;
        } else if (Math.abs(distanceUsed - junctionPreferredFrontDistance) > FRONT_DS_DISTANCE_THRESHOLD) {
            velY += Math.min(MAX_Y_POWER, forwardDistancePID.update(distanceUsed));
        }
    }

    /*public double getRegulatedFrontDistance() {
        double distance = getFrontDistance();
        if (distance > FRONT_DS_MAX) {
            // Keep distance same as last
        } else {
            storedDistance = distance;
        }
        return storedDistance;
    }*/

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
        multiTelemetry.addData("claw_out is active", vertical_arm_to_intake.isActive());
        multiTelemetry.addData("claw_out index", vertical_arm_to_intake.index);
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