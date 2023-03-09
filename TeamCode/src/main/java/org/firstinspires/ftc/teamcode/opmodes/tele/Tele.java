package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.cog.actions.Scheduler;
import org.firstinspires.ftc.teamcode.cog.component.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.cog.component.drive.TileCalculation;
import org.firstinspires.ftc.teamcode.cog.control.GamepadPlus;
import org.firstinspires.ftc.teamcode.cog.opmodes.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.component.slide.VerticalSlide;

@TeleOp(name = "Tele", group = "tele")
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

    @Override
    public void init() {
        //super.init();
        this.robot = new Robot(hardwareMap, telemetry);
        robot.verticalClaw.open();
        this.drivetrain = robot.drivetrain;
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
        update();

        if (pad1.right_trigger_active()) {
            int move_direction = pad1.left_stick_octant();
            if (move_direction != -1)
                queueMoveDirection = move_direction;
            else
                queueMovement(queueMoveDirection);
        } else {
            if (pad1.left_stick_octant() != -1)
                queueMovement(pad1.left_stick_octant());
            else
                queueMovement(queueMoveDirection);
            
            queueMoveDirection = -1;
            double velY = -pad1.gamepad.left_stick_y;
            double velX = pad1.gamepad.left_stick_x;
            double theta = pad1.gamepad.right_stick_x;

            double normalFactor = Math.max(Math.abs(velY) + Math.abs(velX) + Math.abs(theta), 1);
            double frontRight_Power = (velY - velX - theta) / normalFactor;
            double backRight_Power = (velY + velX - theta) / normalFactor;
            double frontLeft_Power = (velY + velX + theta) / normalFactor;
            double backLeft_Power = (velY - velX + theta) / normalFactor;

            drivetrain.setMotorPowers(frontLeft_Power, backLeft_Power, backRight_Power, frontRight_Power);
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
        multiTelemetry.addData("Turret pos", robot.turret.getPosition());
        multiTelemetry.addData("Lever pos", robot.lever.getPosition());
        multiTelemetry.addData("Hinge pos", robot.hinge.getPosition());
        multiTelemetry.update();
    }
}