package incognito.teamcode.opmodes.tele;

import static incognito.teamcode.config.CameraConstants.JUNCTION_HORIZONTAL_DISTANCE_THRESHOLD;
import static incognito.teamcode.config.CameraConstants.JUNCTION_MAX_WIDTH;
import static incognito.teamcode.config.CameraConstants.JUNCTION_MIN_WIDTH;
import static incognito.teamcode.config.CameraConstants.JUNCTION_THETA_POWER_FACTOR;
import static incognito.teamcode.config.CameraConstants.JUNCTION_Y_POWER_FACTOR;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_AVERAGE;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_CONE_OFFSET;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_HIGH;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_LOW;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_MAX;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_MEDIUM;
import static incognito.teamcode.config.GenericConstants.JUNCTION_DISTANCE_PID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import incognito.cog.hardware.gamepad.GamepadPlus;
import incognito.cog.opmodes.RobotOpMode;
import incognito.cog.util.Generic;
import incognito.cog.util.PIDController;
import incognito.cog.util.TelemetryBigError;
import incognito.teamcode.robot.Robot;
import incognito.teamcode.robot.TileMovementPretty;
import incognito.teamcode.robot.WorldRobot;
import incognito.teamcode.robot.component.arm.VerticalArm;
import incognito.teamcode.robot.component.camera.AutoCamera;

@TeleOp(name = "Junction Test", group = "tele")
public class TeleJunction extends RobotOpMode {
    double junctionDistance;
    double junctionArea;
    MultipleTelemetry multiTelemetry;
    AutoCamera camera;
    GamepadPlus pad1;

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

    boolean targetLocking = false;

    PIDController junctionDistancePID = new PIDController(JUNCTION_DISTANCE_PID);

    @Override
    public void init() {
        multiTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        //camera = new AutoCamera(hardwareMap, multiTelemetry);
        robot = new WorldRobot(hardwareMap, telemetry, true);
        pad1 = new GamepadPlus(gamepad1);
        TelemetryBigError.initialize(multiTelemetry);
        robot.autoCamera.swapDoingJunctions();
    }

    @Override
    public void init_loop() {
        multiTelemetry.addData("Junction distance", robot.autoCamera.getJunctionDistance());
        multiTelemetry.addData("Junction area", robot.autoCamera.getJunctionWidth());
        multiTelemetry.addData("Segment index", robot.drivetrain.getCurrentSegmentIndex());
        multiTelemetry.addData("Last segment index", robot.drivetrain.getLastSegmentIndex());
        multiTelemetry.update();
    }

    @Override
    public void loop() {

        if (pad1.a_pressed) {
            // TOGGLE TARGET LOCKING
            targetLocking = !targetLocking;
            if (targetLocking) {
                robot.autoCamera.startCamera();
            } else {
                robot.autoCamera.stopCamera();
            }
        }

        if (pad1.dpad_down_pressed) {
            robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE);
        }
        if (pad1.dpad_left_pressed) {
            robot.verticalArm.goToPosition(VerticalArm.Position.LOW);
        }
        if (pad1.dpad_up_pressed) {
            robot.verticalArm.goToPosition(VerticalArm.Position.MEDIUM);
        }
        if (pad1.dpad_right_pressed) {
            robot.verticalArm.goToPosition(VerticalArm.Position.HIGH);
        }

        if (targetLocking)
            targetLock();
        else
            adjustDrivetrain();


        double normalFactor = Math.max(Math.abs(velY) + Math.abs(velX) + Math.abs(theta), 1);
        double frontRight_Power = (velY - velX - theta) / normalFactor;
        double backRight_Power = (velY + velX - theta) / normalFactor;
        double frontLeft_Power = (velY + velX + theta) / normalFactor;
        double backLeft_Power = (velY - velX + theta) / normalFactor;

        robot.drivetrain.setMotorPowers(frontLeft_Power, backLeft_Power, backRight_Power, frontRight_Power);

        multiTelemetry.update();

        robot.update();
        pad1.update();
    }

    public double getFrontDistance() {
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

    public void targetLock() {
        junctionHorizontalDistance = robot.autoCamera.getJunctionDistance();

        junctionDistancePID.setDesiredValue(0);
        double rotationPower = junctionDistancePID.update(junctionHorizontalDistance);
        if (Math.abs(junctionHorizontalDistance) < JUNCTION_HORIZONTAL_DISTANCE_THRESHOLD) {
            theta = 0;
        } else {
            theta = rotationPower; // junctionHorizontalDistance * JUNCTION_THETA_POWER_FACTOR;
        }
        velX = pad1.gamepad.left_stick_x;
        velY = -pad1.gamepad.left_stick_y;
        // Only move forward once we are locked on horizontally to the junction if using REV sensor
        //if (theta == 0) {
        double junctionMinDistance = getPreferredJunctionDistance();
        if (getFrontDistance() < getPreferredJunctionDistance() || getFrontDistance() > FRONT_DS_MAX) {
            velY += 0;
        } else {
            // Add an amount of power proportional to the distance from the junction
            //  to the robot
            velY += (getFrontDistance() - junctionMinDistance) / (FRONT_DS_MAX - junctionMinDistance);
        }
        //}

        /*
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
        */

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
}