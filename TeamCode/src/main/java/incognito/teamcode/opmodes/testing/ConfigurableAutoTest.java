package incognito.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import incognito.cog.hardware.component.drive.Drivetrain;
import incognito.cog.hardware.component.drive.PoseStorage;
import incognito.cog.opmodes.RobotLinearOpMode;
import incognito.cog.trajectory.TrajectorySequence;
import incognito.teamcode.config.DriveConstants;
import incognito.teamcode.robot.WorldRobot;
import incognito.teamcode.testPackageRemoveLater.ConfigurableTrajectorySequenceBuilder;
import incognito.teamcode.testPackageRemoveLater.ConfigurableTrajectorySequenceContainer;

@Config
@Autonomous(name = "ConfigurableAutoTest", group = "testing")
public class ConfigurableAutoTest extends RobotLinearOpMode {
    WorldRobot robot;
    MultipleTelemetry multiTelemetry;
    public static ConfigurableTrajectorySequenceContainer CTSC = new ConfigurableTrajectorySequenceContainer();
    public static double TEST = 0.0;
    public static PIDFCoefficients TEST2 = new PIDFCoefficients(1, 2, 3, 4);

    public static void createTrajectories() {
        if (CTSC.done()) return; // Not sure if this is necessary but might be nice
        CTSC.trajectorySequenceBuilder("pose1", PoseStorage.lastPose)
                .splineToSplineHeading(
                        new Pose2d(36, -24, Math.toRadians(90)),
                        Math.toRadians(90)
                );
        CTSC.end(); // Not sure if this is necessary but might be nice
    }

    @Override
    public void runOpMode() {
        this.robot = new WorldRobot(hardwareMap, telemetry, true);
        drivetrain = robot.drivetrain;

        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        createTrajectories(); // failsafe?
        waitForStart();
        TrajectorySequence pose1 = CTSC.get("pose1").build();
        multiTelemetry.addData("CTSC", CTSC);
        if (isStopRequested()) return;

        if (pose1 != null)  {
            drivetrain.followTrajectorySequenceAsync(pose1);
        }
        while (drivetrain.isBusy()) {
            drivetrain.update();
            telemetry.addData("Pose", drivetrain.getPoseEstimate());
            telemetry.update();
        }
    }
}
