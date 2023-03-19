package incognito.teamcode.opmodes.tele;

import static incognito.teamcode.config.CameraConstants.JUNCTION_DISTANCE_THRESHOLD;
import static incognito.teamcode.config.CameraConstants.JUNCTION_MAX_WIDTH;
import static incognito.teamcode.config.CameraConstants.JUNCTION_MIN_WIDTH;
import static incognito.teamcode.config.CameraConstants.JUNCTION_THETA_POWER_FACTOR;
import static incognito.teamcode.config.CameraConstants.JUNCTION_Y_POWER_FACTOR;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import incognito.cog.actions.Scheduler;
import incognito.cog.hardware.component.drive.Drivetrain;
import incognito.cog.hardware.component.drive.PoseStorage;
import incognito.cog.hardware.component.drive.TileCalculation;
import incognito.cog.hardware.gamepad.GamepadPlus;
import incognito.cog.opmodes.RobotOpMode;
import incognito.cog.trajectory.TrajectorySequence;
import incognito.teamcode.robot.Robot;
import incognito.teamcode.robot.component.slide.VerticalSlide;

@TeleOp(name = "Tele New Testing", group = "tele")
public class TeleNewTesting extends RobotOpMode {
    // Instance Variables
    protected Robot robot;
    protected Drivetrain drivetrain;
    GamepadPlus pad1;
    GamepadPlus pad2;
    MultipleTelemetry multiTelemetry;
    Scheduler scheduler;

    boolean overrideMain = false;
    boolean levellingEnabled = true;
    boolean yRetraction = false;

    int queueMoveDirection = -1;
    TileCalculation t;
    boolean trajectoryRunning = false;

    boolean targetLocking;

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
        this.robot = new Robot(hardwareMap, telemetry, true);
        robot.autoCamera.swapMode(); // enable target locking mode
        robot.verticalClaw.open();
        this.drivetrain = robot.drivetrain;
        drivetrain.setPoseEstimate(PoseStorage.lastPose);
        pad1 = new GamepadPlus(gamepad1);
        pad2 = new GamepadPlus(gamepad2);
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        scheduler = new Scheduler();
        t = new TileCalculation(robot.drivetrain);
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

        if (pad1.x_pressed && !trajectoryRunning) {
            // Assert we are centered on the current tile
            Vector2d position = t.getVectorByID();
            double heading = drivetrain.getPoseEstimate().getHeading();
            drivetrain.setPoseEstimate(new Pose2d(position, heading));
        }

        if (pad1.left_stick_button_pressed)
            t.queueCenter();

        // TOGGLE X to extend horizontal slide, grab a cone, and retract
        if (pad2.x_pressed) {
            if  (!robot.isExtending && !robot.nearCone()) {
                // Start extend
                robot.extend();
            } else if (robot.isExtending && !robot.nearCone()) {
                // Cancel extend
                // Same thing as if it reaches the cone, except the claw will not close
                // and the arm will not retract
                robot.horizontalScheduler.clearLinear();
                robot.stopExtend();
                robot.retractArm(false, false);
            } else if (robot.isExtending) {
                // At cone distance, close claw and retract
                robot.finishExtend();
            }
        }

        // Enable override, return to only front slide functionality
        if (pad2.left_stick_button_pressed) {
            overrideMain = !overrideMain;
            robot.horizontalSlide.setPower(0);
        }

        // Reset schedules
        if (pad2.right_stick_button_pressed) {
            robot.verticalScheduler.clearGlobal();
            robot.verticalScheduler.clearLinear();
            robot.horizontalScheduler.clearGlobal();
            robot.horizontalScheduler.clearLinear();
            robot.isRetracting = false;
            robot.isDepositing = false;
        }


        // TOGGLE A to toggle back claw
        if (pad2.a_pressed && !overrideMain) {
            robot.horizontalClaw.toggle();
        }

        // TOGGLE B to toggle front claw
        if (pad2.b_pressed) {
            robot.verticalClaw.toggle();
        }


        robot.lever.advancePositionDiscrete(pad2.get_partitioned_right_stick_x());


        /*
        // PRESS A to retract
        if (pad2.a_pressed && !overrideMain) {
            robot.retractArm();
        }

        // PRESS Y to retract and hold, PRESS Y again to drop and return
        if (pad2.y_pressed) {
            if (!yRetraction) {
                robot.retractArm(false, false);
                yRetraction = true;
            } else {
                robot.horizontalClaw.open();
                scheduler.linearSchedule(
                        when -> true,
                        then -> {
                            robot.returnOut();
                            yRetraction = false;
                        },
                        500
                );
            }
        }

        // HOLD TRIGGER to flip
        if (!overrideMain) {
            if (!robot.isDepositing) {
                if (pad2.right_trigger_active()) {
                    robot.verticalFlip.flipDown();
                } else {
                    robot.verticalFlip.flipUp();
                }
            }
        } else {
            if (pad2.right_trigger_pressed) {
                robot.verticalFlip.toggle();
            }
        }
         */

        if (pad2.dpad_down_pressed) {
            multiTelemetry.addLine("Down pressed");
            robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.GROUND);
        }

        if (pad2.dpad_left_pressed) {
            multiTelemetry.addLine("Left pressed");
            robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.LOW);
        }

        if (pad2.dpad_up_pressed) {
            multiTelemetry.addLine("Up pressed");
            robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.MEDIUM);
        }

        if (pad2.dpad_right_pressed) {
            multiTelemetry.addLine("Right pressed");
            robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.HIGH);
        }

        /*
        if (pad2.right_bumper_pressed && !overrideMain) {
            robot.depositCone();
        }
         */

        /*
        // Hinge control, temporary
        if (pad2.left_bumper_pressed) {
            //robot.hinge.setOffsetFactor(robot.hinge.getOffsetFactor() * -1);
            levellingEnabled = !levellingEnabled;
            if (!levellingEnabled)
                robot.hinge.goToSetPosition(0);
        }
        //robot.hinge.offsetPosition(pad2.gamepad.left_trigger);

         */

        robot.verticalSlide.setPower(-pad2.get_partitioned_left_stick_y());
        if (!overrideMain) {
            if (!robot.isRetracting && !robot.isExtending) {
                robot.horizontalSlide.setPower(pad2.get_partitioned_left_stick_x());
                robot.moveLever(-pad2.get_partitioned_right_stick_y(), !yRetraction);
                if (levellingEnabled && !yRetraction)
                    robot.levelHinge();
            }
            // robot.moveTurret(pad2.get_partitioned_right_stick_x());
        }

        fullTelemetry();
    }

    void queueMovement(int move_direction) {
        if (move_direction % 2 == 0) {
            switch (move_direction / 2) {
                case 0:
                    t.queueMove(TileCalculation.Move.RIGHT);
                    break;
                case 1:
                    t.queueMove(TileCalculation.Move.UP);
                    break;
                case 2:
                    t.queueMove(TileCalculation.Move.LEFT);
                    break;
                case 3:
                    t.queueMove(TileCalculation.Move.DOWN);
                    break;
            }
        } else {
            switch ((move_direction + 1) / 2) {
                case 1:
                    t.queueMoveToJunction(TileCalculation.Junction.TOP_RIGHT);
                    break;
                case 2:
                    t.queueMoveToJunction(TileCalculation.Junction.TOP_LEFT);
                    break;
                case 3:
                    t.queueMoveToJunction(TileCalculation.Junction.BOTTOM_LEFT);
                    break;
                case 4:
                    t.queueMoveToJunction(TileCalculation.Junction.BOTTOM_RIGHT);
                    break;
            }
        }
    }

    public void adjustDrivetrain() {
        if (pad1.right_trigger_active()) {
            // Tile based movement mode
            int move_direction = pad1.left_stick_octant();
            if (move_direction != -1)
                queueMoveDirection = move_direction;
            else if (queueMoveDirection != -1) {
                queueMovement(queueMoveDirection);
                queueMoveDirection = move_direction;
            }
            multiTelemetry.addData("Direction: ", move_direction);
        } else {
            queueMoveDirection = -1;
            double inputVelY = -pad1.gamepad.left_stick_y;
            double inputVelX = pad1.gamepad.left_stick_x;
            double inputTheta = pad1.gamepad.right_stick_x;

            double rampSpeed = 0.1;

            velY = ramp(inputVelY, velY, rampSpeed, rampSpeed * 3);
            velX = ramp(inputVelX, velX, rampSpeed * 2, rampSpeed * 3);
            theta = ramp(inputTheta, theta, 0.8);

            if (velX + velY != 0) {
                //trajectoryRunning = false;
                // TODO: Can set trajectory running to false if we implement cancellable trajectories
                t.queueClear();
            }

            double normalFactor = Math.max(Math.abs(velY) + Math.abs(velX) + Math.abs(theta), 1);
            double frontRight_Power = (velY - velX - theta) / normalFactor;
            double backRight_Power = (velY + velX - theta) / normalFactor;
            double frontLeft_Power = (velY + velX + theta) / normalFactor;
            double backLeft_Power = (velY - velX + theta) / normalFactor;

            if (!trajectoryRunning) {
                drivetrain.setMotorPowers(frontLeft_Power, backLeft_Power, backRight_Power, frontRight_Power);
            } else {
                // TODO: TEST
                t.finalizeTrajectory();
                multiTelemetry.addLine("Finalized trajectory!");
            }
        }

        // 1. Have a trajectory and nothing has been started recently
        // 2. Have a trajectory and the current trajectory is finished
        // 3.

        if (t.queueHasTrajectory()) {
            if (!trajectoryRunning) {
                TrajectorySequence sequence = t.build();
                if (sequence != null) {
                    drivetrain.followTrajectorySequenceAsync(sequence);
                    trajectoryRunning = true;
                }
            } else {
                TrajectorySequence sequence = t.build();
                if (sequence != null) {
                    drivetrain.modifyTrajectorySequenceAsync(sequence);
                }
            }
        }

        if (!drivetrain.isBusy()) {
            trajectoryRunning = false;
            t.reset();
        }



        /*
        if (!drivetrain.isBusy() && !trajectoryRunning) {
            if (t.queueHasTrajectory()) {
                Trajectory trajectory = t.queueGet(0);
                drivetrain.followTrajectoryAsync(trajectory);
                trajectoryRunning = true;
            }
        }

        if (!drivetrain.isBusy() && trajectoryRunning) {
            if (t.queueLength() >= 2) {
                t.queueRemove(0);
                Trajectory trajectory = t.queueGet(0);
                drivetrain.followTrajectoryAsync(trajectory);
                trajectoryRunning = true;
            } else if (t.queueLength() == 1) {
                t.queueRemove(0);
            } else {
                trajectoryRunning = false;
            }
        }

         */
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

    public void targetLock() {

        junctionWidth = robot.autoCamera.getJunctionWidth();
        junctionHorizontalDistance = robot.autoCamera.getJunctionDistance();
        // TODO: adjust JUNCTION_Y_POWER_FACTOR so the robot moves quickly when
        //  far away from the junction but slowly when close.
        junctionYPower = 1 / junctionWidth * JUNCTION_Y_POWER_FACTOR;

        // TODO: Test, and consider removing. My thinking is that having the ability
        //  to move the robot in and out along the the autolock for precise movements
        //  could be good.
        velY = pad1.get_partitioned_left_stick_y();

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
        if (Math.abs(junctionHorizontalDistance) < JUNCTION_DISTANCE_THRESHOLD) {
            theta = 0;
        } else {
            theta = junctionHorizontalDistance * JUNCTION_THETA_POWER_FACTOR;
        }

        multiTelemetry.addData("Junction distance", junctionHorizontalDistance);
        multiTelemetry.addData("Junction area", junctionWidth);
        multiTelemetry.addData("Junction Y power", velY);
        multiTelemetry.addData("Junction theta power", theta);


        normalFactor = Math.max(Math.abs(velY) + Math.abs(velX) + Math.abs(theta), 1);
        frontRight_Power = (velY - velX - theta) / normalFactor;
        backRight_Power = (velY + velX - theta) / normalFactor;
        frontLeft_Power = (velY + velX + theta) / normalFactor;
        backLeft_Power = (velY - velX + theta) / normalFactor;

        drivetrain.setMotorPowers(frontLeft_Power, backLeft_Power, backRight_Power, frontRight_Power);
    }

    public void update() {
        robot.update();
        drivetrain.updatePoseEstimate();
        pad1.update();
        pad2.update();
        scheduler.update();
        if (!trajectoryRunning && !drivetrain.isBusy())
            t.update();
        if (drivetrain.isBusy()) {
            drivetrain.update();
        }
    }

    public void fullTelemetry() {
        multiTelemetry.addData("Horizontal motor encoder", robot.horizontalSlide.getPosition());
        multiTelemetry.addData("Vertical motor encoder", robot.verticalSlide.getPosition());
        multiTelemetry.addData("Limit switch:", robot.verticalSlide.getLimitState());
        multiTelemetry.addData("Vertical velocity", robot.verticalSlide.getVelocity());
        multiTelemetry.addData("Partitioned Left Y", pad2.get_partitioned_left_stick_y());
        multiTelemetry.addData("Partitioned Left X", pad2.get_partitioned_left_stick_x());
        multiTelemetry.addData("Partitioned Right Y", pad2.get_partitioned_right_stick_y());
        multiTelemetry.addData("Partitioned Right X", pad2.get_partitioned_right_stick_x());
        // multiTelemetry.addData("Vertical max power", robot.verticalSlide.maxPower);
        // multiTelemetry.addData("Vertical direction", robot.verticalSlide.getDirection());
        // multiTelemetry.addData("Vertical Powers", robot.verticalSlide.getPowers());
        multiTelemetry.addLine();
        multiTelemetry.addData("Vertical stopDir", robot.verticalSlide.stopDirection);
        multiTelemetry.addData("Horizontal stopDir", robot.horizontalSlide.stopDirection);
        multiTelemetry.addData("Retracting", robot.isRetracting);
        multiTelemetry.addData("Depositing", robot.isDepositing);
        // multiTelemetry.addData("Target", robot.horizontalSlide.motors[0].getTargetPosition());
        // multiTelemetry.addData("Vel", robot.horizontalSlide.motors[0].getVelocity());
        // multiTelemetry.addData("Power", robot.horizontalSlide.motors[0].getPower());
        // multiTelemetry.addData("Mode", robot.horizontalSlide.motors[0].getMode());
        multiTelemetry.addLine();
        // multiTelemetry.addData("Turret pos", robot.turret.getPosition());
        multiTelemetry.addData("Lever pos", robot.lever.getPosition());
        multiTelemetry.addData("Hinge pos", robot.hinge.getPosition());
        multiTelemetry.addLine();
        multiTelemetry.addData("Pose", drivetrain.getPoseEstimate());
        multiTelemetry.addData("Distance (cm)", robot.horizontalDistanceSensor.getDistance(DistanceUnit.CM));

        multiTelemetry.update();
    }
}