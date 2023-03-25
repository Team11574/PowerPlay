package incognito.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ConcurrentModificationException;
import java.util.concurrent.TimeUnit;

import incognito.cog.actions.Scheduler;
import incognito.cog.opmodes.RobotLinearOpMode;
import incognito.cog.trajectory.TrajectorySequence;
import incognito.teamcode.robot.Robot;
import incognito.teamcode.robot.component.servoImplementations.Lever;
import incognito.teamcode.robot.component.slide.VerticalSlide;

@Autonomous(name = "AUTO Right 3 Cone", group = "auto", preselectTeleOp = "Tele")
public class AutoRight3Cone extends RobotLinearOpMode {
    // Instance variables
    MultipleTelemetry multiTelemetry;
    Scheduler scheduler;
    TrajectorySequence[] spots;
    TrajectorySequence initialPos;
    TrajectorySequence readjustPos;
    TrajectorySequence moveLeft;
    TrajectorySequence moveRight;
    ElapsedTime timer;
    int parkingSpot = 2;

    @Override
    public void runOpMode() {
        try {
            //super.runOpMode();

            // drive.trajectorySequenceBuilder(new Pose2d(36, -61.5, Math.toRadians(90)))
            // Forward 57, left 2, rotate 160 degrees

            multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


            this.robot = new Robot(hardwareMap, multiTelemetry, true);

            drivetrain = robot.drivetrain;


            scheduler = new Scheduler();

            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

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
                    .strafeLeft(14)
                    .build();

            moveRight = drivetrain.trajectorySequenceBuilder(moveLeft.end())
                    .strafeRight(14)
                    .build();

            readjustPos = drivetrain.trajectorySequenceBuilder(initialPos.end())
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
            parkingSpot = robot.getParkingSpot();
        } catch (ConcurrentModificationException e) {
            multiTelemetry.addData("Concurrent Mod Exception:", e.toString());
        }

        if (isStopRequested()) return;

        initialPositionAndCone();

        moveToPoleAndBack(Lever.LeverPosition.FIFTH, true);
        moveToPoleAndBack(Lever.LeverPosition.FOURTH, false);

        drivetrain.followTrajectorySequenceAsync(readjustPos);
        while (drivetrain.isBusy()) {
            drivetrain.update();
            robot.update();
        }
        park(parkingSpot);

        multiTelemetry.addLine("Yippee!");
        multiTelemetry.update();
    }

    private void initialPositionAndCone() {

        //robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.HIGH);
        // GO TO INITIAL POS
        drivetrain.followTrajectorySequenceAsync(initialPos);
        // START GOING UP
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.HIGH);
        // Wait til at position and fully up
        while (drivetrain.isBusy() || !robot.verticalSlide.atSetPosition()) {
            if (drivetrain.isBusy()) {
                drivetrain.update();
            }
            robot.update();
            scheduler.update();
        }
        // First deposit
        robot.depositCone();
        // Queue horizontal slide out
        robot.horizontalSlide.setTargetPosition(1450);
        robot.lever.goToSetPosition(Lever.LeverPosition.MID);
        robot.levelHinge();
        // Wait for deposit finish
        robot.waitForDeposit();
        // Queue vertical slide down
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.GROUND);
    }

    private void moveToPoleAndBack(Lever.LeverPosition height, boolean oneMore) {
        // Queue first move to stack
        drivetrain.followTrajectorySequenceAsync(moveLeft);
        // Wait until in position, horizontal slide is out (vertical slide not necessarily down)
        while (drivetrain.isBusy() || !robot.horizontalSlide.atSetPosition()) {
            if (drivetrain.isBusy())
                drivetrain.update();
            robot.update();
            scheduler.update();
            multiTelemetry.addData("IsBusy", drivetrain.isBusy());
            multiTelemetry.addData("NotAtVerticalPos", !robot.verticalSlide.atSetPosition());
            multiTelemetry.addData("VerticalTarget", robot.verticalSlide.getTargetPosition());
            multiTelemetry.addData("NotAtHorizontalPos", !robot.horizontalSlide.atSetPosition());
            multiTelemetry.update();
        }

        // Adjust arm
        robot.lever.goToSetPosition(height);
        robot.levelHinge();
        nap(350);
        // Grab cone
        robot.horizontalClaw.close();
        nap(250);
        // Queue horizontal retraction without drop
        robot.retractArm(false, false);

        // Queue moving back
        scheduler.globalSchedule(
                when -> true,
                then -> drivetrain.followTrajectorySequenceAsync(moveRight),
                500
        );

        // Wait until retracted, vertical slide at ground, and moved in position
        while (robot.isRetracting || scheduler.hasGlobalQueries() || drivetrain.isBusy()) { // || !robot.verticalSlide.atSetPosition()) {
            if (drivetrain.isBusy()) {
                drivetrain.update();
            }
            robot.update();
            scheduler.update();

            multiTelemetry.addData("IsRetracting", robot.isRetracting);
            multiTelemetry.addData("IsBusy", drivetrain.isBusy());
            multiTelemetry.addData("HasQueries", scheduler.hasGlobalQueries());
            multiTelemetry.addData("NotAtVerticalPos", !robot.verticalSlide.atSetPosition());
            multiTelemetry.addData("VerticalTarget", robot.verticalSlide.getTargetPosition());
            multiTelemetry.addData("VerticalPos", robot.verticalSlide.getPosition());
            multiTelemetry.update();
        }
        robot.update();
        // Swap cone
        //sleep(250);
        robot.horizontalClaw.open();
        nap(350);
        // Swap cone
        robot.verticalClaw.close();
        // Queue vertical slide up
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.HIGH);
        // Queue horizontal slide out if want one more cone
        if (oneMore) {
            scheduler.globalSchedule(
                    when -> true,
                    then -> {
                        robot.returnOut();
                        robot.lever.goToSetPosition(Lever.LeverPosition.MID);
                        robot.levelHinge();
                    },
                    500
            );
        }

        // Wait until at height
        while (!robot.verticalSlide.atSetPosition()) {
            robot.update();
            scheduler.update();
        }
        // Deposit second cone
        robot.depositCone();
        robot.waitForDeposit();
        // Queue vertical slide to ground
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.GROUND);

        /*
        // Queue drivetrain to move towards stack
        drivetrain.followTrajectorySequenceAsync(moveLeft);
        // Wait until horizontal slide is ready and drivetrain is in position
        while (scheduler.hasGlobalQueries() || drivetrain.isBusy() ||
                !robot.horizontalSlide.atSetPosition()) {
            if (drivetrain.isBusy()) {
                drivetrain.update();
            }
            robot.update();
            scheduler.update();
        }
        // Adjust arm
        robot.lever.goToSetPosition(Lever.LeverPosition.FOURTH);
        robot.levelHinge();
        sleep(350);
        // Grab cone
        robot.horizontalClaw.close();
        sleep(250);
        // Queue retraction
        robot.retractArm(false, false);

        // Queue move to pole
        scheduler.globalSchedule(
                when -> true,
                then -> drivetrain.followTrajectorySequenceAsync(moveRight),
                500
        );

        // Wait until retracted and at pole
        while(robot.isRetracting || drivetrain.isBusy() ||
                scheduler.hasGlobalQueries() || !robot.verticalSlide.atSetPosition()) {
            if (drivetrain.isBusy()) {
                drivetrain.update();
            }
            robot.update();
            scheduler.update();
        }




        sleep(250);
        robot.horizontalClaw.open();
        sleep(250);
        // Grab second cone
        robot.verticalClaw.close();
        // Queue vertical slide up
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.HIGH);
        // Queue horizontal slide out
        scheduler.globalSchedule(
                when -> true,
                then -> {
                    robot.returnOut();
                    robot.lever.goToSetPosition(Lever.LeverPosition.MID);
                    robot.levelHinge();
                },
                500
        );

        // Wait until at height
        while (!robot.verticalSlide.atSetPosition()) {
            robot.update();
            scheduler.update();
        }
        // Deposit second cone
        robot.depositCone();
        robot.waitForDeposit();
        // Queue vertical slide to ground
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.GROUND);
        // Queue drivetrain to move towards stack
        drivetrain.followTrajectorySequenceAsync(moveLeft);
        // Wait until horizontal slide is ready and drivetrain is in position
        while (scheduler.hasGlobalQueries() || drivetrain.isBusy() ||
                !robot.horizontalSlide.atSetPosition()) {
            if (drivetrain.isBusy()) {
                drivetrain.update();
            }
            robot.update();
            scheduler.update();
        }
        // Adjust arm
        robot.lever.goToSetPosition(Lever.LeverPosition.THIRD);
        robot.levelHinge();
        sleep(350);
        // Grab third stack cone
        robot.horizontalClaw.close();
        sleep(250);
        // Queue retraction
        robot.retractArm(false, false);

        // Queue move to pole
        scheduler.globalSchedule(
                when -> true,
                then -> drivetrain.followTrajectorySequenceAsync(moveRight),
                500
        );

        // Wait until retracted and at pole
        while(robot.isRetracting || drivetrain.isBusy() ||
                scheduler.hasGlobalQueries() || !robot.verticalSlide.atSetPosition()) {
            if (drivetrain.isBusy()) {
                drivetrain.update();
            }
            robot.update();
            scheduler.update();
        }

        sleep(250);
        robot.horizontalClaw.open();
        sleep(250);
        // Grab third stack cone inner
        robot.verticalClaw.close();
        // Queue vertical slide up
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.HIGH);
        // Queue horizontal slide out
        scheduler.globalSchedule(
                when -> true,
                then -> {
                    robot.returnOut();
                    robot.lever.goToSetPosition(Lever.LeverPosition.MID);
                    robot.levelHinge();
                },
                500
        );

        // Wait until at height
        while (!robot.verticalSlide.atSetPosition()) {
            robot.update();
            scheduler.update();
        }
        // Deposit second cone
        robot.depositCone();
        robot.waitForDeposit();
        // Queue vertical slide to ground
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.GROUND);
         */
        //runCones();
        //sleep(8000);
    }

    void park(int parkingSpot) {
        multiTelemetry.addLine("Attempting to park in space " + parkingSpot);
        if (!isStopRequested())
            drivetrain.followTrajectorySequence(spots[parkingSpot - 1]);

    }

    void nap(double milliseconds) {
        timer.reset();
        while (timer.time(TimeUnit.MILLISECONDS) <= milliseconds) {
            robot.update();
        }
    }

    /*

    void nap(double milliseconds) {
        nap(milliseconds, 5);
    }

    void nap(double milliseconds, double refreshTime) {
        double remainder = milliseconds % refreshTime;
        for (int i = 1; i <= milliseconds/refreshTime; i++) {
            robot.update();
            sleep((long) refreshTime);
        }
        if (remainder != 0)
            sleep((long) remainder);
    }
     */
}