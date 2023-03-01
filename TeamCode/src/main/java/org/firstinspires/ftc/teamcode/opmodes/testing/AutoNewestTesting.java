package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.base.RobotLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.component.claw.Lever;
import org.firstinspires.ftc.teamcode.robot.component.slide.VerticalSlide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.runnable.Scheduler;

@Autonomous(name = "AUTO Newest Testing", group = "testing")
public class AutoNewestTesting extends RobotLinearOpMode {
    // Instance variables
    MultipleTelemetry multiTelemetry;
    Scheduler scheduler;
    TrajectorySequence[] spots;
    TrajectorySequence initialPos;
    TrajectorySequence moveLeft;
    TrajectorySequence moveRight;

    @Override
    public void runOpMode() {
        //super.runOpMode();

        // drive.trajectorySequenceBuilder(new Pose2d(36, -61.5, Math.toRadians(90)))
        // Forward 57, left 2, rotate 160 degrees

        this.robot = new Robot(hardwareMap, telemetry, true);

        drivetrain = robot.drivetrain;

        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        scheduler = new Scheduler();

        //Pose2d startPos = new Pose2d(36, 62.5, Math.toRadians(270));
        Pose2d startPos = new Pose2d(36, -62.5, Math.toRadians(90));
        //Pose2d startPos = new Pose2d(-36, 5.5, Math.toRadians(270));
        //Pose2d startPos = new Pose2d(0, 0, 0);//Math.toRadians(270));
        drivetrain.setPoseEstimate(startPos);
        initialPos = drivetrain.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(36, 1, Math.toRadians(182)))
                .lineToLinearHeading(new Pose2d(33, 1, Math.toRadians(182)))
                .build();
        // first cone stuff
        moveLeft = drivetrain.trajectorySequenceBuilder(initialPos.end())
                .strafeLeft(13.5)
                .build();

        moveRight = drivetrain.trajectorySequenceBuilder(moveLeft.end())
                .strafeRight(13.5)
                .build();

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
        robot.autoCamera.terminateCamera();

        int parkingSpot = robot.getParkingSpot();

        if(isStopRequested()) return;

        //robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.HIGH);
        drivetrain.followTrajectorySequence(initialPos);
        //sleep(2000);
        robot.horizontalSlide.setTargetPosition(1450);
        robot.lever.goToSetPosition(Lever.LeverPosition.MID);
        robot.levelHinge();

        drivetrain.followTrajectorySequenceAsync(moveLeft);
        while(drivetrain.isBusy() || !robot.horizontalSlide.atSetPosition()) {
            drivetrain.update();
            robot.update();
            //scheduler.update();
        }

        robot.lever.goToSetPosition(Lever.LeverPosition.FIFTH);
        robot.levelHinge();
        sleep(350);
        robot.horizontalClaw.close();
        sleep(250);
        robot.retractArm(false, true);

        scheduler.globalSchedule(
                when -> true,
                then -> drivetrain.followTrajectorySequenceAsync(moveRight),
                500
        );

        while(robot.isRetracting || drivetrain.isBusy() || scheduler.hasGlobalQueries()) {
            drivetrain.update();
            robot.update();
            scheduler.update();
        }

        sleep(350);
        robot.verticalClaw.close();
        scheduler.globalSchedule(
                when -> true,
                then -> {
                    robot.returnOut();
                    robot.lever.goToSetPosition(Lever.LeverPosition.MID);
                    robot.levelHinge();
                },
                500
        );

        drivetrain.followTrajectorySequenceAsync(moveLeft);
        while (scheduler.hasGlobalQueries() || drivetrain.isBusy() || !robot.horizontalSlide.atSetPosition()) {
            drivetrain.update();
            robot.update();
            scheduler.update();
        }
        robot.verticalClaw.open();
        robot.lever.goToSetPosition(Lever.LeverPosition.FOURTH);
        robot.levelHinge();
        sleep(350);
        robot.horizontalClaw.close();
        sleep(250);
        robot.retractArm(false, true);

        scheduler.globalSchedule(
                when -> true,
                then -> drivetrain.followTrajectorySequenceAsync(moveRight),
                500
        );

        while(robot.isRetracting || drivetrain.isBusy() || scheduler.hasGlobalQueries()) {
            drivetrain.update();
            robot.update();
            scheduler.update();
        }

        sleep(350);
        robot.verticalClaw.close();
        //runCones();
        //sleep(8000);

        drivetrain.followTrajectorySequence(readjustPos);
        park(parkingSpot);

        multiTelemetry.addLine("Yippee!");
        multiTelemetry.update();

    }

    void runCones() {
        robot.depositCone();
        robot.lever.goToSetPosition(Lever.LeverPosition.FIFTH); // 5th cone height
        robot.levelHinge();
        robot.horizontalSlide.setTargetPosition(1630);
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
        sleep(250);
        robot.retractArm(false, true);
        robot.waitForRetract();
        sleep(350);
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
        sleep(350);
        robot.retractArm(false, false);

        while (!robot.verticalSlide.atSetPosition()) {
            // wait
            robot.update();
        }
        robot.waitForRetract(); // double check horizontal is completed
        sleep(500);
        robot.horizontalClaw.open();
        sleep(250);
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
        sleep(350);
        robot.retractArm(false, false);

        while (!robot.verticalSlide.atSetPosition()) {
            // wait
            robot.update();
        }
        robot.waitForRetract(); // double check horizontal is completed
        sleep(500);
        robot.horizontalClaw.open();
        sleep(250);
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
}
