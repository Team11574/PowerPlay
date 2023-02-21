package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.RobotLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.Drivetrain;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AUTO Left Park", group = "auto", preselectTeleOp = "Tele")
public class Auto extends RobotLinearOpMode {
    // Instance variables
    TrajectorySequence spot1;
    TrajectorySequence spot2;
    TrajectorySequence spot3;

    TrajectorySequence[] spots;


    @Override
    public void runOpMode() {
        //super.runOpMode();
        this.robot = new Robot(hardwareMap, telemetry, true);

        drivetrain = robot.getDrivetrain();

        Pose2d startPos = new Pose2d(0,0,0);
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

        int parkingSpot = robot.getParkingSpot();

        park(parkingSpot);

    }

    void park(int parkingSpot) {
        telemetry.addLine("Attempting to park in space " + parkingSpot);
        if (!isStopRequested())
            drivetrain.followTrajectorySequence(spots[parkingSpot - 1]);

    }
}
