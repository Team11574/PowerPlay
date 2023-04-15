package incognito.teamcode.opmodes.tele;

import static incognito.teamcode.robot.TileMovement.BuildState.ACTIVE;
import static incognito.teamcode.robot.TileMovement.BuildState.INACTIVE;
import incognito.teamcode.robot.TileMovement.MoveDirection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;

import incognito.cog.hardware.component.drive.Drivetrain;
import incognito.cog.hardware.component.drive.PoseStorage;
import incognito.cog.hardware.component.drive.TileCalculation;
import incognito.cog.hardware.gamepad.GamepadPlus;
import incognito.cog.opmodes.RobotOpMode;
import incognito.cog.util.TelemetryBigError;
import incognito.teamcode.robot.Robot;
import incognito.teamcode.robot.TileMovement;
import incognito.teamcode.robot.WorldRobot;

@TeleOp(name = "Tile Test", group = "testing")
public class TeleTile extends RobotOpMode {
    MultipleTelemetry multiTelemetry;
    GamepadPlus pad1;
    WorldRobot robot;
    Drivetrain drivetrain;
    TileMovement tileMovement;
    ArrayList<TileMovement.BuildState> buildStates = new ArrayList<>();
    int queueMoveDirection = -1;

    @Override
    public void init() {
        multiTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        //camera = new AutoCamera(hardwareMap, multiTelemetry);
        robot = new WorldRobot(hardwareMap, telemetry, false);
        drivetrain = robot.drivetrain;
        drivetrain.setPoseEstimate(PoseStorage.lastPose);
        pad1 = new GamepadPlus(gamepad1);
        tileMovement = new TileMovement(drivetrain);
        TelemetryBigError.initialize(multiTelemetry);
    }

    @Override
    public void loop() {
        /* Things to test:
            - distance to each height of junction
            - direction towards ground junction
         */

        if (pad1.a_pressed) {
            // Queue some moves
            tileMovement.move(TileMovement.MoveDirection.UP)
                .move(TileMovement.MoveDirection.UP)
                .move(TileMovement.MoveDirection.LEFT);
        }

        if (pad1.dpad_up_pressed) {
            tileMovement.move(TileMovement.MoveDirection.UP);
        }
        if (pad1.dpad_down_pressed) {
            tileMovement.move(TileMovement.MoveDirection.DOWN);
        }
        if (pad1.dpad_left_pressed) {
            tileMovement.move(TileMovement.MoveDirection.LEFT);
        }
        if (pad1.dpad_right_pressed) {
            tileMovement.move(TileMovement.MoveDirection.RIGHT);
        }


        if (pad1.x_pressed) {
            // Queue some moves
            TelemetryBigError.raise(1);
        }


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
            if (pad1.left_stick_button_pressed) {
                tileMovement.moveCenter();
            }
        } else {
            queueMoveDirection = -1;
        }
        if (pad1.right_trigger_depressed) {
            // If we have just finished queueing some actions, execute them
            drivetrain.followTrajectorySequenceAsync(tileMovement.build());
            tileMovement.reset();
        }

        if (pad1.b_pressed) {
            tileMovement.reset();
        }

        TelemetryBigError.update();
        multiTelemetry.addData("Current segment index", drivetrain.getCurrentSegmentIndex());
        multiTelemetry.addData("Build index", tileMovement.getBuildIndex());
        multiTelemetry.addData("Last built index", tileMovement.getLastBuiltIndex());
        multiTelemetry.addData("Last direction", tileMovement.getLastDirection());
        multiTelemetry.addData("Sequence start pose", tileMovement.getSequenceStartPose());
        multiTelemetry.addData("Build states", buildStates);
        multiTelemetry.addData("Sequence size", drivetrain.getCurrentTrajectorySize());


        drivetrain.updatePoseEstimate();
        if (drivetrain.isBusy()) {
            drivetrain.update();
        }

        multiTelemetry.update();
        robot.update();
        pad1.update();
    }

    TileMovement queueMovement(int move_direction) {
        if (move_direction % 2 == 0) {
            switch (move_direction / 2) {
                case 0: return tileMovement.move(MoveDirection.RIGHT);
                case 1: return tileMovement.move(MoveDirection.UP);
                case 2: return tileMovement.move(MoveDirection.LEFT);
                case 3: return tileMovement.move(MoveDirection.DOWN);
            }
        } else {
            switch ((move_direction + 1) / 2) {
                case 1: return tileMovement.moveToJunction(MoveDirection.J_UP_RIGHT);
                case 2: return tileMovement.moveToJunction(MoveDirection.J_UP_LEFT);
                case 3: return tileMovement.moveToJunction(MoveDirection.J_DOWN_LEFT);
                case 4: return tileMovement.moveToJunction(MoveDirection.DOWN_RIGHT);
            }
        }
        // never happens but Java is mad if I don't have a default return
        return tileMovement;
    }
}