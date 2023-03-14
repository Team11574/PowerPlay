package incognito.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import incognito.cog.opmodes.RobotLinearOpMode;
import incognito.cog.trajectory.TrajectorySequence;
import incognito.teamcode.robot.Robot;

@Autonomous(name = "AUTO Park", group = "auto", preselectTeleOp = "Tele")
public class AutoPark extends RobotLinearOpMode {
    // Instance variables
    TrajectorySequence spot1;
    TrajectorySequence spot2;
    TrajectorySequence spot3;

    TrajectorySequence[] spots;


    @Override
    public void runOpMode() {
        super.runOpMode();

        this.robot = new Robot(hardwareMap, telemetry, true);

        drivetrain = robot.drivetrain;

        Pose2d startPos = new Pose2d(0, 0, 0);
        spot1 = drivetrain.trajectorySequenceBuilder(startPos)
                .forward(28)
                .strafeLeft(24)
                .build();

        spot2 = drivetrain.trajectorySequenceBuilder(startPos)
                .forward(28)
                .build();

        spot3 = drivetrain.trajectorySequenceBuilder(startPos)
                .forward(28)
                .strafeRight(-24)
                .build();

        spots = new TrajectorySequence[]{spot1, spot2, spot3};

        waitForStart();
        robot.autoCamera.stopCamera();

        int parkingSpot = robot.getParkingSpot();

        park(parkingSpot);

    }

    void park(int parkingSpot) {
        telemetry.addLine("Attempting to park in space " + parkingSpot);
        if (!isStopRequested())
            drivetrain.followTrajectorySequence(spots[parkingSpot - 1]);

    }
}