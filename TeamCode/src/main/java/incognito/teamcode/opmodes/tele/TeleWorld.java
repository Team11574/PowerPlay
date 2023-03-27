package incognito.teamcode.opmodes.tele;

import static incognito.teamcode.config.CameraConstants.JUNCTION_HORIZONTAL_DISTANCE_THRESHOLD;
import static incognito.teamcode.config.CameraConstants.JUNCTION_MAX_WIDTH;
import static incognito.teamcode.config.CameraConstants.JUNCTION_MIN_WIDTH;
import static incognito.teamcode.config.CameraConstants.JUNCTION_THETA_POWER_FACTOR;
import static incognito.teamcode.config.CameraConstants.JUNCTION_Y_POWER_FACTOR;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import incognito.cog.actions.Scheduler;
import incognito.cog.hardware.component.drive.Drivetrain;
import incognito.cog.hardware.component.drive.PoseStorage;
import incognito.cog.hardware.component.drive.TileCalculation;
import incognito.cog.hardware.gamepad.GamepadPlus;
import incognito.cog.opmodes.RobotOpMode;
import incognito.cog.trajectory.TrajectorySequence;
import incognito.cog.util.AsciiTrajectory;
import incognito.cog.util.TelemetryBigError;
import incognito.teamcode.robot.TileMovementPretty;
import incognito.teamcode.robot.WorldRobot;
import incognito.teamcode.robot.component.arm.VerticalArm;
import incognito.teamcode.robot.component.slide.VerticalSlide;

@TeleOp(name = "Tele WORLD", group = "tele")
public class TeleWorld extends RobotOpMode {
    // Instance Variables
    protected WorldRobot robot;
    protected Drivetrain drivetrain;
    GamepadPlus pad1;
    GamepadPlus pad2;
    MultipleTelemetry multiTelemetry;
    Scheduler scheduler;

    boolean overrideMain = false;
    boolean levellingEnabled = true;
    boolean yRetraction = false;

    int queueMoveDirection = -1;
    TileMovementPretty tileMovement;
    boolean trajectoryRunning = false;

    boolean targetLocking = false;

    double velX, velY, theta = 0;
    double junctionWidth;
    double junctionHorizontalDistance;
    double junctionYPower;
    double normalFactor;
    double frontRight_Power;
    double backRight_Power;
    double frontLeft_Power;
    double backLeft_Power;

    @Override
    public void init() {
        //super.init();
        this.robot = new WorldRobot(hardwareMap, telemetry, true);
        robot.autoCamera.setDoingJunctions(true); // prep target locking mode
        this.drivetrain = robot.drivetrain;
        drivetrain.setPoseEstimate(PoseStorage.lastPose);
        pad1 = new GamepadPlus(gamepad1);
        pad2 = new GamepadPlus(gamepad2);
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        scheduler = new Scheduler();
        tileMovement = new TileMovementPretty(robot.drivetrain, new AsciiTrajectory());
        TelemetryBigError.initialize(multiTelemetry);
    }

    @Override
    public void loop() {
        //super.loop();
        update();

        // Adjust drivetrain
        if (!targetLocking) {
            adjustDrivetrain();
        } else {
            targetLock();
        }

        if (pad1.a_pressed) {
            // TOGGLE TARGET LOCKING
            targetLocking = !targetLocking;
            if (targetLocking) {
                robot.autoCamera.startCamera();
            } else {
                robot.autoCamera.stopCamera();
            }
        }

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

        // update();
        // multiTelemetry.addData("Front distance (cm):", "%.2f cm", robot.frontDistanceSensor.getDistance(DistanceUnit.CM));
        multiTelemetry.addData("Front distance (cm): cm", robot.frontDistanceSensor.getDistance(DistanceUnit.CM));
        multiTelemetry.update();
        // fullTelemetry();
    }

    TileMovementPretty queueMovement(int move_direction) {
        if (move_direction % 2 == 0) {
            switch (move_direction / 2) {
                case 0: return tileMovement.move(TileMovementPretty.MoveDirection.RIGHT);
                case 1: return tileMovement.move(TileMovementPretty.MoveDirection.UP);
                case 2: return tileMovement.move(TileMovementPretty.MoveDirection.LEFT);
                case 3: return tileMovement.move(TileMovementPretty.MoveDirection.DOWN);
            }
        } else {
            switch ((move_direction + 1) / 2) {
                case 1: return tileMovement.moveToJunction(TileMovementPretty.MoveDirection.J_UP_RIGHT);
                case 2: return tileMovement.moveToJunction(TileMovementPretty.MoveDirection.J_UP_LEFT);
                case 3: return tileMovement.moveToJunction(TileMovementPretty.MoveDirection.J_DOWN_LEFT);
                case 4: return tileMovement.moveToJunction(TileMovementPretty.MoveDirection.DOWN_RIGHT);
            }
        }
        // never happens but Java is mad if I don't have a default return
        return tileMovement;
    }

    public void adjustDrivetrain() {
        if (pad1.right_trigger_pressed) {
            drivetrain.updatePoseEstimate();
            tileMovement.resetTrajectoryOutput();
        }
        if (pad1.right_trigger_depressed) {
            // If we have just finished queueing some actions, execute them
            if (tileMovement.hasMoves()) {
                drivetrain.updatePoseEstimate();
                drivetrain.followTrajectorySequenceAsync(tileMovement.build());
            }
            tileMovement.reset();
        }
        if (pad1.right_trigger_active()) {
            // Cancel trajectory mid queue
            if (pad1.b_pressed) {
                tileMovement.reset();
            }
            if (pad1.left_stick_button_pressed) {
                tileMovement.moveCenter();
            }
            // Tile based movement mode
            int move_direction = pad1.left_stick_octant();
            if (move_direction != -1)
                queueMoveDirection = move_direction;
            else if (queueMoveDirection != -1) {
                queueMovement(queueMoveDirection);
                queueMoveDirection = move_direction;
            }
            multiTelemetry.addLine(tileMovement.getTrajectoryOutput());
            multiTelemetry.addLine();
            multiTelemetry.addLine(AsciiTrajectory.octantOutput(move_direction));
            multiTelemetry.addLine();
        } else if (!drivetrain.isBusy()) {
            queueMoveDirection = -1;
            double inputVelY = -pad1.gamepad.left_stick_y;
            double inputVelX = pad1.gamepad.left_stick_x;
            double inputTheta = pad1.gamepad.right_stick_x;

            double rampSpeed = 0.1;

            velY = ramp(inputVelY, velY, rampSpeed, rampSpeed * 3);
            velX = ramp(inputVelX, velX, rampSpeed * 2, rampSpeed * 3);
            theta = ramp(inputTheta, theta, 0.8);

            normalFactor = Math.max(Math.abs(velY) + Math.abs(velX) + Math.abs(theta), 1);
            frontRight_Power = (velY - velX - theta) / normalFactor;
            backRight_Power = (velY + velX - theta) / normalFactor;
            frontLeft_Power = (velY + velX + theta) / normalFactor;
            backLeft_Power = (velY - velX + theta) / normalFactor;

            drivetrain.setMotorPowers(frontLeft_Power, backLeft_Power, backRight_Power, frontRight_Power);
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
        if (Math.abs(junctionHorizontalDistance) < JUNCTION_HORIZONTAL_DISTANCE_THRESHOLD) {
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

    public void update() {
        TelemetryBigError.update();
        robot.update();
        pad1.update();
        pad2.update();
        scheduler.update();
        if (drivetrain.isBusy()) {
            drivetrain.update();
        }
    }

    public void fullTelemetry() {
        multiTelemetry.addData("Vertical slide motor encoder", robot.verticalArm.slide.getPosition());
        multiTelemetry.addData("Vertical slide target position", robot.verticalArm.slide.getTargetPosition());
        multiTelemetry.addData("Vertical slide motor power", robot.verticalArm.slide.getPower());
        multiTelemetry.addData("Vertical slide limit switch:", robot.verticalArm.slide.getLimitState());
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
        multiTelemetry.addData("Pose", drivetrain.getPoseEstimate());
        multiTelemetry.addData("Distance (cm)", robot.horizontalDistanceSensor.getDistance(DistanceUnit.CM));

        multiTelemetry.update();
    }
}