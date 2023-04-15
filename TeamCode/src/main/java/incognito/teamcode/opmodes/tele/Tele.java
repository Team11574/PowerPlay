package incognito.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import incognito.cog.actions.Scheduler;
import incognito.cog.hardware.component.drive.Drivetrain;
import incognito.cog.hardware.component.drive.PoseStorage;
import incognito.cog.hardware.component.drive.TileCalculation;
import incognito.cog.hardware.gamepad.GamepadPlus;
import incognito.cog.opmodes.RobotOpMode;
import incognito.cog.trajectory.TrajectorySequence;
import incognito.teamcode.robot.Robot;
import incognito.teamcode.robot.component.slide.VerticalSlide;

@TeleOp(name = "Tele", group = "old")
public class Tele extends RobotOpMode {
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

    @Override
    public void init() {
        //super.init();
        this.robot = new Robot(hardwareMap, telemetry);
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
        robot.update();
        drivetrain.updatePoseEstimate();
        update();

        if (pad1.right_trigger_active()) {
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
            double velY = -pad1.gamepad.left_stick_y;
            double velX = pad1.gamepad.left_stick_x;
            double theta = pad1.gamepad.right_stick_x;

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

            if (!trajectoryRunning)
                drivetrain.setMotorPowers(frontLeft_Power, backLeft_Power, backRight_Power, frontRight_Power);
        }

        // 1. Have a trajectory and nothing has been started recently
        // 2. Have a trajectory and the current trajectory is finished
        // 3.


        if (t.queueHasTrajectory()) {
            TrajectorySequence sequence = t.build();
            if (!trajectoryRunning) {
                if (sequence != null) {
                    drivetrain.followTrajectorySequenceAsync(sequence);
                    trajectoryRunning = true;
                }
            } else {
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

        if (pad1.left_stick_button_pressed)
            t.queueCenter();

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


        // TOGGLE X to toggle back claw
        if (pad2.x_pressed && !overrideMain) {
            robot.horizontalClaw.toggle();
        }

        // TOGGLE B to toggle front claw
        if (pad2.b_pressed) {
            robot.verticalClaw.toggle();
        }

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

        if (pad2.dpad_down_pressed) {
            multiTelemetry.addLine("Down pressed");
            robot.verticalSlide.goToSetPosition(VerticalSlide.Position.INTAKE);
        }

        if (pad2.dpad_left_pressed) {
            multiTelemetry.addLine("Left pressed");
            robot.verticalSlide.goToSetPosition(VerticalSlide.Position.LOW);
        }

        if (pad2.dpad_up_pressed) {
            multiTelemetry.addLine("Up pressed");
            robot.verticalSlide.goToSetPosition(VerticalSlide.Position.MEDIUM);
        }

        if (pad2.dpad_right_pressed) {
            multiTelemetry.addLine("Right pressed");
            robot.verticalSlide.goToSetPosition(VerticalSlide.Position.HIGH);
        }

        if (pad2.right_bumper_pressed && !overrideMain) {
            robot.depositCone();
        }

        // Hinge control, temporary
        if (pad2.left_bumper_pressed) {
            //robot.hinge.setOffsetFactor(robot.hinge.getOffsetFactor() * -1);
            levellingEnabled = !levellingEnabled;
            if (!levellingEnabled)
                robot.hinge.goToSetPosition(0);
        }
        //robot.hinge.offsetPosition(pad2.gamepad.left_trigger);

        robot.verticalSlide.setPower(-pad2.get_partitioned_left_stick_y());
        if (!overrideMain) {
            if (!robot.isRetracting) {
                robot.horizontalSlide.setPower(pad2.get_partitioned_left_stick_x());
                robot.moveLever(-pad2.get_partitioned_right_stick_y(), !yRetraction);
                if (levellingEnabled && !yRetraction)
                    robot.levelHinge();
            }
            robot.moveTurret(pad2.get_partitioned_right_stick_x());
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

    public void update() {
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
        multiTelemetry.addData("Limit switch:", robot.verticalSlide.getDangerState());
        multiTelemetry.addData("Vertical velocity", robot.verticalSlide.getVelocity());
        multiTelemetry.addData("Partitioned Left Y", pad2.get_partitioned_left_stick_y());
        multiTelemetry.addData("Partitioned Left X", pad2.get_partitioned_left_stick_x());
        multiTelemetry.addData("Partitioned Right Y", pad2.get_partitioned_right_stick_y());
        multiTelemetry.addData("Partitioned Right X", pad2.get_partitioned_right_stick_x());
        // multiTelemetry.addData("Vertical max power", robot.verticalSlide.maxPower);
        // multiTelemetry.addData("Vertical direction", robot.verticalSlide.getDirection());
        // multiTelemetry.addData("Vertical Powers", robot.verticalSlide.getPowers());
        multiTelemetry.addLine();
        multiTelemetry.addData("Retracting", robot.isRetracting);
        multiTelemetry.addData("Depositing", robot.isDepositing);
        // multiTelemetry.addData("Target", robot.horizontalSlide.motors[0].getTargetPosition());
        // multiTelemetry.addData("Vel", robot.horizontalSlide.motors[0].getVelocity());
        // multiTelemetry.addData("Power", robot.horizontalSlide.motors[0].getPower());
        // multiTelemetry.addData("Mode", robot.horizontalSlide.motors[0].getMode());
        multiTelemetry.addLine();
        multiTelemetry.addData("Turret pos", robot.turret.getPosition());
        multiTelemetry.addData("Lever pos", robot.lever.getPosition());
        multiTelemetry.addData("Hinge pos", robot.hinge.getPosition());
        multiTelemetry.addLine();
        multiTelemetry.addData("Pose", drivetrain.getPoseEstimate());

        multiTelemetry.update();
    }
}