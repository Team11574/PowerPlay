package incognito.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import incognito.cog.hardware.gamepad.GamepadPlus;
import incognito.cog.opmodes.RobotOpMode;
import incognito.cog.trajectory.TrajectorySequence;
import incognito.teamcode.robot.Robot;
import incognito.teamcode.robot.component.camera.AutoCamera;

@TeleOp(name = "Junction Test", group = "tele")
public class TeleJunction extends RobotOpMode {
    double junctionDistance;
    double junctionArea;
    MultipleTelemetry multiTelemetry;
    AutoCamera camera;
    Robot robot;
    double velX;
    double velY;
    double theta;
    GamepadPlus pad1;

    boolean targetLocking = true;

    @Override
    public void init() {
        multiTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        //camera = new AutoCamera(hardwareMap, multiTelemetry);
        robot = new Robot(hardwareMap, telemetry, true);
        pad1 = new GamepadPlus(gamepad1);

        robot.autoCamera.swapMode();
    }

    @Override
    public void init_loop() {
        multiTelemetry.addData("Junction distance", robot.autoCamera.getJunctionDistance());
        multiTelemetry.addData("Junction area", robot.autoCamera.getJunctionArea());
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

    private void targetLock() {
        junctionDistance = robot.autoCamera.getJunctionDistance();
        junctionArea = robot.autoCamera.getJunctionArea();
        multiTelemetry.addData("Junction distance", junctionDistance);
        multiTelemetry.addData("Junction area", junctionArea);
        double adjustedArea = 800 / junctionArea;
        double junctionMaxArea = 3000;
        multiTelemetry.addData("Adjusted area", adjustedArea);
        if (junctionArea > junctionMaxArea) {
            // TODO: Consider adding in negative velocity if too close to the pole
            // Happens when robot is about 11.5 cm from the pole
            velY = 0;
        } else {
            velY = Math.max(0, adjustedArea);
        }
        velX = pad1.gamepad.left_stick_x;
        if (Math.abs(junctionDistance) < 10) {
            theta = 0;
        } else {
            theta = -0.005 * junctionDistance;
        }
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