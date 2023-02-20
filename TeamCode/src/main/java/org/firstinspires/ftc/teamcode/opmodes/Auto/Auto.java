package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.RobotLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.components.Drivetrain;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AUTO Park", group = "auto")
public class Auto extends RobotLinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = robot.getDrivetrain();

        Pose2d startPos = new Pose2d(0,0,0);
        TrajectorySequence t1 = drivetrain.trajectorySequenceBuilder(startPos)
                .forward(20)
                .strafeLeft(20)
                .build();

        waitForStart();

        if (!isStopRequested())
            drivetrain.followTrajectorySequence(t1);
    }
}
