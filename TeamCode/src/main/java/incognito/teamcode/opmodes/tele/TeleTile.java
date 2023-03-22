package incognito.teamcode.opmodes.tele;

import static incognito.teamcode.config.CameraConstants.JUNCTION_HORIZONTAL_DISTANCE_THRESHOLD;
import static incognito.teamcode.config.CameraConstants.JUNCTION_MAX_WIDTH;
import static incognito.teamcode.config.CameraConstants.JUNCTION_MIN_WIDTH;
import static incognito.teamcode.config.CameraConstants.JUNCTION_THETA_POWER_FACTOR;
import static incognito.teamcode.config.CameraConstants.JUNCTION_Y_POWER_FACTOR;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import incognito.cog.hardware.component.drive.Drivetrain;
import incognito.cog.hardware.component.drive.PoseStorage;
import incognito.cog.hardware.gamepad.GamepadPlus;
import incognito.cog.opmodes.RobotOpMode;
import incognito.cog.util.TelemetryBigError;
import incognito.teamcode.robot.Robot;
import incognito.teamcode.robot.TileCalculationBetter;
import incognito.teamcode.robot.TileCalculationBetter2;
import incognito.teamcode.robot.component.camera.AutoCamera;

@TeleOp(name = "Tile Test", group = "tele")
public class TeleTile extends RobotOpMode {
    MultipleTelemetry multiTelemetry;
    GamepadPlus pad1;
    Robot robot;
    Drivetrain drivetrain;
    TileCalculationBetter2 tileCalculation;

    @Override
    public void init() {
        multiTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        //camera = new AutoCamera(hardwareMap, multiTelemetry);
        robot = new Robot(hardwareMap, telemetry, false);
        drivetrain = robot.drivetrain;
        drivetrain.setPoseEstimate(PoseStorage.lastPose);
        pad1 = new GamepadPlus(gamepad1);
        tileCalculation = new TileCalculationBetter2(drivetrain);
        TelemetryBigError.initialize(multiTelemetry);
    }

    @Override
    public void loop() {

        if (pad1.a_pressed) {
            // Queue some moves
            tileCalculation.move(TileCalculationBetter2.TileDirection.UP);
            tileCalculation.move(TileCalculationBetter2.TileDirection.UP);
            tileCalculation.move(TileCalculationBetter2.TileDirection.LEFT);
        }

        if (pad1.x_pressed) {
            // Queue some moves
            TelemetryBigError.raise(1);
        }

        switch (tileCalculation.getBuildState(drivetrain.getCurrentSegmentIndex())) {
            case STARTED:
                drivetrain.followTrajectorySequenceAsync(tileCalculation.build());
                break;
            case ADJUSTED:
                drivetrain.modifyTrajectorySequenceAsync(tileCalculation.build());
                break;
            case ACTIVE:
                // dont go to manual mode
                break;
            case INACTIVE:
                // stay in manual mode?
                break;
            case FINISHED:
                // go to manual mode?
                break;
            case ERROR:
                multiTelemetry.addData("Error", "Error building trajectory");
                break;
        }

        if (pad1.b_pressed) {
            tileCalculation.reset();
        }

        TelemetryBigError.update();
        multiTelemetry.addData("Current segment index", drivetrain.getCurrentSegmentIndex());
        multiTelemetry.addData("Build index", tileCalculation.getBuildIndex());
        multiTelemetry.addData("Last built index", tileCalculation.getLastBuiltIndex());
        multiTelemetry.addData("Build state", tileCalculation.getBuildState(drivetrain.getCurrentSegmentIndex()));
        multiTelemetry.addData("Last direction", tileCalculation.getLastDirection());
        multiTelemetry.addData("Sequence start pose", tileCalculation.getSequenceStartPose());

        multiTelemetry.update();
        robot.update();
        pad1.update();
        drivetrain.updatePoseEstimate();
        if (drivetrain.isBusy()) {
            drivetrain.update();
        }
    }
}