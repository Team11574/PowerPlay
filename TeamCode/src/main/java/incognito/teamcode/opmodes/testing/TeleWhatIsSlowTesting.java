package incognito.teamcode.opmodes.testing;

import static incognito.teamcode.config.CameraConstants.JUNCTION_MAX_WIDTH;
import static incognito.teamcode.config.CameraConstants.JUNCTION_MIN_HORIZONTAL_DISTANCE;
import static incognito.teamcode.config.CameraConstants.JUNCTION_MIN_WIDTH;
import static incognito.teamcode.config.CameraConstants.JUNCTION_THETA_POWER_FACTOR;
import static incognito.teamcode.config.CameraConstants.JUNCTION_Y_POWER_FACTOR;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import incognito.cog.hardware.component.drive.PoseStorage;
import incognito.cog.hardware.gamepad.GamepadPlus;
import incognito.cog.opmodes.RobotOpMode;
import incognito.cog.util.TelemetryBigError;
import incognito.teamcode.robot.WorldRobot;
import incognito.teamcode.robot.component.arm.VerticalArm;
import incognito.teamcode.robot.component.camera.AutoCamera;

@Disabled
@TeleOp(name = "AGH WHAT IS SLOW Test", group = "tele")
public class TeleWhatIsSlowTesting extends RobotOpMode {
    double junctionDistance;
    double junctionArea;
    MultipleTelemetry multiTelemetry;
    AutoCamera camera;
    GamepadPlus pad1;
    GamepadPlus pad2;

    WorldRobot robot;
    double velX;
    double velY;
    double theta;
    double junctionWidth;
    double junctionHorizontalDistance;
    double junctionYPower;
    double normalFactor;
    double frontRight_Power;
    double backRight_Power;
    double frontLeft_Power;
    double backLeft_Power;

    boolean targetLocking = true;

    @Override
    public void init() {
        robot = new WorldRobot(hardwareMap, telemetry, true);
        drivetrain = robot.drivetrain;
        drivetrain.setPoseEstimate(PoseStorage.lastPose);
        pad1 = new GamepadPlus(gamepad1);
        pad2 = new GamepadPlus(gamepad2);
        multiTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        robot.autoCamera.swapDoingJunctions();
        TelemetryBigError.initialize(multiTelemetry);
    }

    @Override
    public void init_loop() {
        multiTelemetry.addData("Junction distance", robot.autoCamera.getJunctionDistance());
        multiTelemetry.addData("Junction area", robot.autoCamera.getJunctionWidth());
        multiTelemetry.update();
    }

    @Override
    public void loop() {

        update();

        if (pad1.a_pressed) {
            // TOGGLE TARGET LOCKING
            targetLocking = !targetLocking;
            if (targetLocking) {
                robot.autoCamera.startCamera();
            } else {
                robot.autoCamera.stopCamera();
            }
        }

        if (targetLocking)
            targetLock();
        else
            adjustDrivetrain();

        // TOGGLE B to toggle front claw
        if (pad2.b_pressed) {
            robot.verticalArm.toggleClaw();
        }

        if (pad2.dpad_down_pressed) {
            robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE);
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

        // HOLD right trigger to move down hinge
        if (pad2.right_trigger_pressed) {
            robot.verticalArm.hingeDown();
        }
        if (pad2.right_trigger_depressed) {
            robot.verticalArm.hingeUp();
        }

        if (!drivetrain.isBusy()) {
            double normalFactor = Math.max(Math.abs(velY) + Math.abs(velX) + Math.abs(theta), 1);
            double frontRight_Power = (velY - velX - theta) / normalFactor;
            double backRight_Power = (velY + velX - theta) / normalFactor;
            double frontLeft_Power = (velY + velX + theta) / normalFactor;
            double backLeft_Power = (velY - velX + theta) / normalFactor;

            robot.drivetrain.setMotorPowers(frontLeft_Power, backLeft_Power, backRight_Power, frontRight_Power);
        }

        update();
    }

    public void update() {
        TelemetryBigError.update();
        //fullTelemetry();
        multiTelemetry.update();
        robot.update();
        pad1.update();
        pad2.update();
        if (drivetrain.isBusy()) {
            drivetrain.update();
        }
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
    }

    public void adjustDrivetrain() {
        double inputVelY = -pad1.gamepad.left_stick_y;
        double inputVelX = pad1.gamepad.left_stick_x;
        double inputTheta = pad1.gamepad.right_stick_x;

        double rampSpeed = 0.1;

        velY = ramp(inputVelY, velY, rampSpeed, rampSpeed * 3);
        velX = ramp(inputVelX, velX, rampSpeed * 2, rampSpeed * 3);
        theta = ramp(inputTheta, theta, 0.8);
    }

    private double ramp(double input, double currentValue, double speed) {
        return ramp(input, currentValue, speed, null);
    }

    private double ramp(Double input, Double currentValue, Double speed, Double speedDown) {
        double inputDisplacement = input - currentValue;
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
        //multiTelemetry.addData("Pose", drivetrain.getPoseEstimate());
        //multiTelemetry.addData("Distance (cm)", robot.horizontalDistanceSensor.getDistance(DistanceUnit.CM));
    }
}