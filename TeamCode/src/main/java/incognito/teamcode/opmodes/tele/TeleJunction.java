package incognito.teamcode.opmodes.tele;

import static incognito.teamcode.config.GenericConstants.JUNCTION_MAX_HORIZONTAL_DISTANCE;
import static incognito.teamcode.config.GenericConstants.JUNCTION_MIN_HORIZONTAL_DISTANCE;
import static incognito.teamcode.config.GenericConstants.FORWARD_DISTANCE_PID;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_AVERAGE;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_CONE_OFFSET;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_DISTANCE_THRESHOLD;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_HIGH;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_LOW;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_MAX;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_MEDIUM;
import static incognito.teamcode.config.GenericConstants.FRONT_DS_THETA_THRESHOLD;
import static incognito.teamcode.config.GenericConstants.JUNCTION_DISTANCE_PID;
import static incognito.teamcode.config.GenericConstants.JUNCTION_PREFERRED_PIXEL_DISTANCE_CLOSE;
import static incognito.teamcode.config.GenericConstants.JUNCTION_PREFERRED_PIXEL_DISTANCE_FAR;
import static incognito.teamcode.config.GenericConstants.JUNCTION_THETA_DISTANCE_FACTOR;
import static incognito.teamcode.config.GenericConstants.JUNCTION_WIDTH_TO_DISTANCE_FACTOR;
import static incognito.teamcode.config.GenericConstants.MAX_THETA_POWER;
import static incognito.teamcode.config.GenericConstants.MAX_Y_POWER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import incognito.cog.hardware.gamepad.GamepadPlus;
import incognito.cog.opmodes.RobotOpMode;
import incognito.cog.util.PIDController;
import incognito.cog.util.TelemetryBigError;
import incognito.teamcode.robot.WorldRobot;
import incognito.teamcode.robot.component.arm.VerticalArm;
import incognito.teamcode.robot.component.camera.AutoCamera;

@TeleOp(name = "Junction Test", group = "testing")
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
    PIDController forwardDistancePID = new PIDController(FORWARD_DISTANCE_PID);

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

        if (pad1.right_trigger_pressed) {
            // TOGGLE TARGET LOCKING
            targetLocking = true;
            junctionDistancePID.reset();
            forwardDistancePID.reset();
            robot.autoCamera.startCamera();
        }
        if (pad1.right_trigger_depressed) {
            targetLocking = false;
            robot.autoCamera.stopCamera();
            velX = 0;
            velY = 0;
            theta = 0;
        }

        if (pad1.left_trigger_pressed) {
            robot.verticalArm.hingeDown();
        }
        if (pad1.left_trigger_depressed) {
            robot.verticalArm.hingeUp();
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
        }else if (Math.abs(distanceUsed - junctionPreferredFrontDistance) > FRONT_DS_DISTANCE_THRESHOLD) {
            velY += Math.min(MAX_Y_POWER, forwardDistancePID.update(distanceUsed));
        }

        // Only move forward once we are locked on horizontally to the junction if using REV sensor
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

        multiTelemetry.addData("Junction horizontal distance", junctionHorizontalDistance);
        multiTelemetry.addData("Junction forward pref distance", junctionPreferredFrontDistance);
        multiTelemetry.addData("Junction forward distance sensor", getFrontDistance());
        multiTelemetry.addData("Junction width camera", robot.autoCamera.getJunctionWidth());
        multiTelemetry.addData("Junction forward distance camera", junctionWidthDistance);
        multiTelemetry.addData("Forward Distance used", distanceUsed);
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

        multiTelemetry.addData("X power", velX);
        multiTelemetry.addData("Y power", velY);
        multiTelemetry.addData("Theta power", theta);
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