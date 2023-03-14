package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.cog.actions.Scheduler;
import org.firstinspires.ftc.teamcode.cog.opmodes.RobotLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.component.claw.Lever;
import org.firstinspires.ftc.teamcode.robot.component.slide.VerticalSlide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "AUTO Right 4 Cone", group = "auto", preselectTeleOp = "Tele")
public class AutoRight4Cone extends RobotLinearOpMode {
    // Instance variables
    MultipleTelemetry multiTelemetry;
    Scheduler scheduler;
    TrajectorySequence[] spots;

    ElapsedTime timer;

    @Override
    public void runOpMode() {
        //super.runOpMode();

        // drive.trajectorySequenceBuilder(new Pose2d(36, -61.5, Math.toRadians(90)))
        // Forward 57, left 2, rotate 160 degrees

        this.robot = new Robot(hardwareMap, telemetry, true);

        drivetrain = robot.drivetrain;

        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        scheduler = new Scheduler();

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        //Pose2d startPos = new Pose2d(36, 62.5, Math.toRadians(270));
        Pose2d startPos = new Pose2d(36, -62.5, Math.toRadians(90));
        //Pose2d startPos = new Pose2d(-36, 5.5, Math.toRadians(270));
        //Pose2d startPos = new Pose2d(0, 0, 0);//Math.toRadians(270));
        drivetrain.setPoseEstimate(startPos);
        TrajectorySequence initialPos = drivetrain.trajectorySequenceBuilder(startPos)
                .forward(58)
                .back(3)
                .lineToLinearHeading(new Pose2d(startPos.component1() - 2.5, startPos.component2() + 59, Math.toRadians(170.8)))
                .build();
                /*
                //.lineTo(new Vector2d(-36, 5.5))
                .forward(59)
                .back(2)
                .strafeLeft(2)
                .turn(Math.toRadians(70))
                /*
                .addDisplacementMarker(() -> {
                    multiTelemetry.addData("Pose", drivetrain.getPoseEstimate());
                    multiTelemetry.update();
                })
                .lineTo(new Vector2d(0, -32))
                .addDisplacementMarker(() -> {
                    multiTelemetry.addData("Pose", drivetrain.getPoseEstimate());
                    multiTelemetry.update();
                })
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(4, -32, Math.toRadians(340))) // Math.toRadians(330)))
                .addDisplacementMarker(() -> {
                    multiTelemetry.addData("Pose", drivetrain.getPoseEstimate());
                    multiTelemetry.update();
                })
                //.lineToLinearHeading(new Pose2d(36, -30, Math.toRadians(90)))
                .waitSeconds(2)
                //.lineToLinearHeading(new Pose2d(0, 58 - 3, 0))
                //.lineToLinearHeading(new Pose2d(startPos.component1()-3, startPos.component2() + 57, Math.toRadians(160)))
                */


        TrajectorySequence readjustPos = drivetrain.trajectorySequenceBuilder(initialPos.end())
                //.strafeLeft(3)
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(90)))
                //.back(4)
                .build();

        TrajectorySequence spot1 = drivetrain.trajectorySequenceBuilder(readjustPos.end())
                .strafeLeft(24)
                //.back(21.5)
                .build();

        TrajectorySequence spot2 = drivetrain.trajectorySequenceBuilder(readjustPos.end())
                .back(2)
                //.back(21.5)
                .build();

        TrajectorySequence spot3 = drivetrain.trajectorySequenceBuilder(readjustPos.end())
                .strafeRight(22)
                //.back(21.5)
                .build();

        spots = new TrajectorySequence[]{spot1, spot2, spot3};

        waitForStart();
        robot.autoCamera.stopCamera();

        int parkingSpot = robot.getParkingSpot();

        if (isStopRequested()) return;

        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.HIGH);
        drivetrain.followTrajectorySequence(initialPos);
        runCones();

        drivetrain.followTrajectorySequence(readjustPos);
        park(parkingSpot);

        multiTelemetry.addLine("Yippee!");
        multiTelemetry.update();

    }

    void runCones() {
        robot.depositCone();
        robot.lever.goToSetPosition(Lever.LeverPosition.FIFTH); // 5th cone height
        robot.levelHinge();
        robot.horizontalSlide.setTargetPosition(1620);
        robot.horizontalClaw.open();
        while (robot.isDepositing) {
            robot.update();
        }
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.GROUND);
        robot.verticalClaw.open();
        while (!robot.horizontalSlide.atSetPosition()) {
            // wait
            robot.update();
        }

        robot.horizontalClaw.close();
        nap(250);
        robot.retractArm(false, true);
        robot.waitForRetract();
        nap(350);
        robot.verticalClaw.close();
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.HIGH);
        scheduler.linearSchedule(
                when -> true,
                then -> {
                    robot.returnOut();
                    robot.lever.goToSetPosition(Lever.LeverPosition.FOURTH);
                    robot.levelHinge();
                },
                500
        );
        while (!robot.verticalSlide.atSetPosition()) {
            // wait
            robot.update();
            scheduler.update();
        }
        robot.depositCone();
        robot.waitForDeposit();
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.GROUND);
        robot.horizontalClaw.close();
        nap(350);
        robot.retractArm(false, false);

        while (!robot.verticalSlide.atSetPosition()) {
            // wait
            robot.update();
        }
        robot.waitForRetract(); // double check horizontal is completed
        nap(500);
        robot.horizontalClaw.open();
        nap(250);
        robot.verticalClaw.close();
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.HIGH);
        scheduler.linearSchedule(
                when -> true,
                then -> {
                    robot.returnOut();
                    robot.lever.goToSetPosition(Lever.LeverPosition.THIRD);
                    robot.levelHinge();
                },
                500
        );
        while (!robot.verticalSlide.atSetPosition()) {
            // wait
            robot.update();
            scheduler.update();
        }
        robot.depositCone();
        robot.waitForDeposit();
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.GROUND);
        robot.horizontalClaw.close();
        nap(350);
        robot.retractArm(false, false);

        while (!robot.verticalSlide.atSetPosition()) {
            // wait
            robot.update();
        }
        robot.waitForRetract(); // double check horizontal is completed
        nap(500);
        robot.horizontalClaw.open();
        nap(250);
        robot.verticalClaw.close();
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.HIGH);
        while (!robot.verticalSlide.atSetPosition()) {
            // wait
            robot.update();
        }
        robot.depositCone();
        robot.waitForDeposit();
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.GROUND);
    }

    void park(int parkingSpot) {
        telemetry.addLine("Attempting to park in space " + parkingSpot);
        if (!isStopRequested())
            drivetrain.followTrajectorySequence(spots[parkingSpot - 1]);

    }

    void nap(double milliseconds) {
        timer.reset();
        while (timer.time(TimeUnit.MILLISECONDS) <= milliseconds) {
            robot.update();
        }
    }
}