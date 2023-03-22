package incognito.teamcode.opmodes.tele;

import static incognito.teamcode.config.CameraConstants.JUNCTION_HORIZONTAL_DISTANCE_THRESHOLD;
import static incognito.teamcode.config.CameraConstants.JUNCTION_MAX_WIDTH;
import static incognito.teamcode.config.CameraConstants.JUNCTION_MIN_WIDTH;
import static incognito.teamcode.config.CameraConstants.JUNCTION_THETA_POWER_FACTOR;
import static incognito.teamcode.config.CameraConstants.JUNCTION_Y_POWER_FACTOR;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import incognito.cog.hardware.gamepad.GamepadPlus;
import incognito.cog.opmodes.RobotOpMode;
import incognito.cog.util.TelemetryBigError;
import incognito.teamcode.robot.Robot;
import incognito.teamcode.robot.component.camera.AutoCamera;

@TeleOp(name = "Junction Test", group = "tele")
public class TeleJunction extends RobotOpMode {
    double junctionDistance;
    double junctionArea;
    MultipleTelemetry multiTelemetry;
    AutoCamera camera;
    GamepadPlus pad1;

    Robot robot;
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
        multiTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        //camera = new AutoCamera(hardwareMap, multiTelemetry);
        robot = new Robot(hardwareMap, telemetry, true);
        pad1 = new GamepadPlus(gamepad1);
        TelemetryBigError.initialize(multiTelemetry);
        robot.autoCamera.swapMode();
    }

    @Override
    public void init_loop() {
        multiTelemetry.addData("Junction distance", robot.autoCamera.getJunctionDistance());
        multiTelemetry.addData("Junction area", robot.autoCamera.getJunctionWidth());
        multiTelemetry.addData("Segment index", robot.drivetrain.getCurrentSegmentIndex());
        multiTelemetry.addData("Segment count", robot.drivetrain.getCurrentSegmentSize());
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
        if (Math.abs(junctionHorizontalDistance) < JUNCTION_HORIZONTAL_DISTANCE_THRESHOLD) {
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
}