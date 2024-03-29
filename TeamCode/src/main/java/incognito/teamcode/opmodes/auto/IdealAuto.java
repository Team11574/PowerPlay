package incognito.teamcode.opmodes.auto;

import static incognito.teamcode.config.WorldSlideConstants.VS_CLAW_DROP_SPEED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

import incognito.cog.actions.Action;
import incognito.cog.actions.ActionManager;
import incognito.cog.actions.Scheduler;
import incognito.cog.opmodes.RobotLinearOpMode;
import incognito.cog.trajectory.TrajectorySequence;
import incognito.teamcode.robot.Robot;
import incognito.teamcode.robot.WorldRobot;
import incognito.teamcode.robot.component.arm.HorizontalArm;
import incognito.teamcode.robot.component.arm.VerticalArm;
import incognito.teamcode.robot.component.servoImplementations.Lever;
import incognito.teamcode.robot.component.slide.VerticalSlide;

@Autonomous(name = "Ideal Auto FAKE", group = "old")
public class IdealAuto extends RobotLinearOpMode {
    WorldRobot robot;
    MultipleTelemetry multiTelemetry;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() {
        this.robot = new WorldRobot(hardwareMap, telemetry, true);

        drivetrain = robot.drivetrain;

        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.verticalArm.closeClaw();

        ActionManager.clear();

        double MOVEMENT_DELAY = 250;

        Pose2d startPos = new Pose2d(36, -62.5, Math.toRadians(90));
        drivetrain.setPoseEstimate(startPos);

        TrajectorySequence firstGoToFirstJunction = drivetrain.trajectorySequenceBuilder(startPos) //r.startPose)
                .splineToConstantHeading(
                        new Vector2d(36, -24),
                        Math.toRadians(90)
                )
                .splineToSplineHeading(
                        new Pose2d(35, -11, Math.toRadians(130)),
                        Math.toRadians(90)
                )
                .build();
        TrajectorySequence faceFirstStack = drivetrain.trajectorySequenceBuilder(firstGoToFirstJunction.end())
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(
                        new Pose2d(42, -12, Math.toRadians(180)),
                        Math.toRadians(0)
                )
                .build();
        TrajectorySequence faceFirstJunction = drivetrain.trajectorySequenceBuilder(faceFirstStack.end())
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(
                        new Pose2d(35, -11, Math.toRadians(130)),
                        Math.toRadians(180)
                )
                .build();
        Action toHigh = new Action(robot.verticalArm::closeClaw)
                .then(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.WAIT_OUT))
                //.delay(0)
                .then(() -> robot.verticalArm.goToPosition(VerticalArm.Position.HIGH))
                .until(robot.verticalArm::atPosition)
                .then(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.CLAW_OUT))
                .delay(50)
                .then(robot.verticalArm::openClaw)
                .delay(100)
                //.then(() -> robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE))
                .globalize();
        Action outtake = new Action(() -> robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE))
                .delay(MOVEMENT_DELAY)
                .then(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.OUT))
                // Wait until arm stops going out
                .until(robot.horizontalArm::atPosition)
                .globalize();
        Action intake = new Action(() -> {
            robot.verticalArm.openClaw();
            robot.horizontalArm.goToPosition(HorizontalArm.Position.WAIT_IN);
        })
                .until(() -> robot.verticalArm.atPosition() && robot.horizontalArm.atPosition())
                .then(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.IN))
                .until(robot.horizontalArm::atPosition)
                .then(robot.horizontalArm::openClaw)
                .globalize();

        Action wait_face_junction = new Action()
                .delay(MOVEMENT_DELAY)
                .then(() -> drivetrain.followTrajectorySequenceAsync(faceFirstJunction))
                .globalize();

        /*
                // 1
                .waitSeconds(.25)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(
                        new Pose2d(42, -12, Math.toRadians(180)),
                        Math.toRadians(0)
                )
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(
                        new Pose2d(35, -12, Math.toRadians(130)),
                        Math.toRadians(180)
                )
                // 2
                .waitSeconds(.25)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(
                        new Pose2d(42, -12, Math.toRadians(180)),
                        Math.toRadians(0)
                )
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(
                        new Pose2d(35, -12, Math.toRadians(130)),
                        Math.toRadians(180)
                )
                //3
                .waitSeconds(.25)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(
                        new Pose2d(42, -12, Math.toRadians(180)),
                        Math.toRadians(0)
                )
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(
                        new Pose2d(35, -12, Math.toRadians(130)),
                        Math.toRadians(180)
                )
                // 4
                .waitSeconds(.25)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(
                        new Pose2d(42, -12, Math.toRadians(180)),
                        Math.toRadians(0)
                )
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(
                        new Pose2d(35, -12, Math.toRadians(130)),
                        Math.toRadians(180)
                )
                // 5
                .waitSeconds(.25)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(
                        new Pose2d(42, -12, Math.toRadians(180)),
                        Math.toRadians(0)
                )
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(
                        new Pose2d(35, -12, Math.toRadians(130)),
                        Math.toRadians(180)
                )
                .setTangent(Math.toRadians(180))
                // Center out
                .splineToSplineHeading(
                        new Pose2d(24, -11, Math.toRadians(90)),
                        Math.toRadians(180)
                )
                .splineToSplineHeading(
                        new Pose2d(-24, -12, Math.toRadians(90)),
                        Math.toRadians(180)
                )
                .splineToSplineHeading(
                        new Pose2d(-33.5, -3.5, Math.toRadians(180-170.8)),
                        Math.toRadians(90)
                )
                .waitSeconds(cycleTime * 2)
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(
                        new Pose2d(-12, -12, Math.toRadians(90)),
                        Math.toRadians(0)
                )

                .build();*/
        waitForStart();
        if (isStopRequested()) return;
        drivetrain.followTrajectorySequenceAsync(firstGoToFirstJunction);
        //deposit cone
        while (drivetrain.isBusy()) {
            update();
        }
        robot.horizontalArm.storeLeverHeight(Lever.HorizontalLeverPosition.FIFTH);
        toHigh.run();
        while (toHigh.isActive()) {
            update();
            if (!opModeIsActive()) break;
        }
        drivetrain.followTrajectorySequenceAsync(faceFirstStack);
        outtake.run();
        while (outtake.isActive() || drivetrain.isBusy()) {
            update();
            if (!opModeIsActive()) break;
        }
        wait_face_junction.run();
        intake.run();
        while (intake.isActive() || wait_face_junction.isActive() || drivetrain.isBusy()) {
            update();
            if (!opModeIsActive()) break;
        }
        toHigh.run();
        while (toHigh.isActive()) {
            update();
            if (!opModeIsActive()) break;
        }
        robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE);
        nap(1000);


        /*
        robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE);
        drivetrain.followTrajectorySequence(faceFirstStack);
        nap(1000);
        drivetrain.followTrajectorySequence(faceFirstJunction);

         */
    }

    public void update() {
        ActionManager.update();
        robot.update();
        if (drivetrain.isBusy()) {
            drivetrain.update();
        }
    }

    void nap(double milliseconds) {
        timer.reset();
        while (timer.time(TimeUnit.MILLISECONDS) <= milliseconds) {
            update();
        }
    }
}