package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.RobotLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.slides.Slide;
import org.firstinspires.ftc.teamcode.robot.components.slides.VerticalSlide;
import org.firstinspires.ftc.teamcode.robot.exceptions.UndefinedSetPositionException;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AUTO Cone Left", group = "auto", preselectTeleOp = "Tele")
public class AutoConeLeft extends RobotLinearOpMode {
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


        /*
        drive.trajectorySequenceBuilder(new Pose2d(36, -61.5, Math.toRadians(90)))
            .forward(51.5)
            .strafeLeft(12)
            .addDisplacementMarker(() -> {
                // Raise slides, flip claw, drop cone, etc
            })
            .back(2)
            //.strafeLeft(12)
            //.strafeRight(12)
            .strafeRight(33)
            .build()
         */

        Pose2d startPos = new Pose2d(0,0,0);

        drivetrain.setPoseEstimate(startPos);


        TrajectorySequence cone = drivetrain.trajectorySequenceBuilder(startPos)
                //new Pose2d(36, -61.5, Math.toRadians(90)))
                .forward(53.5)
                .strafeLeft(12.5)
                .addDisplacementMarker(() -> {
                    // Raise slides, flip claw, drop cone, etc

                    try {
                        robot.getVerticalSlide().goToSetPosition(VerticalSlide.SetPosition.HIGH);
                    } catch (UndefinedSetPositionException e) {
                        e.printStackTrace();
                    }

                    telemetry.addLine("Yippee!");
                    telemetry.update();
                })
                .back(2.5)
                .build();

        spot1 = drivetrain.trajectorySequenceBuilder(cone.end())
                .strafeLeft(12.5)
                .back(23)
                .build();

        spot2 = drivetrain.trajectorySequenceBuilder(cone.end())
                .strafeRight(11.5)
                .back(23)
                .build();

        spot3 = drivetrain.trajectorySequenceBuilder(cone.end())
                .strafeRight(35)
                .back(23)
                .build();

        spots = new TrajectorySequence[]{spot1, spot2, spot3};

        waitForStart();

        int parkingSpot = robot.getParkingSpot();

        drivetrain.followTrajectorySequence(cone);

        park(parkingSpot);

    }

    void park(int parkingSpot) {
        telemetry.addLine("Attempting to park in space " + parkingSpot);
        if (!isStopRequested())
            drivetrain.followTrajectorySequence(spots[parkingSpot - 1]);

    }
}
