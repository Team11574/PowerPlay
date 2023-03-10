package incognito.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import incognito.cog.component.drive.SampleMecanumDrive;
import incognito.cog.trajectory.TrajectorySequence;

@Autonomous(name = "Parking Test", group = "Testing")
public class ParkTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPos = new Pose2d(0, 0, 0);
        TrajectorySequence t1 = drive.trajectorySequenceBuilder(startPos)
                .forward(20)
                .strafeLeft(20)
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(t1);
    }

}