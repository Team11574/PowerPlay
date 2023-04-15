package incognito.teamcode.opmodes.auto;

import static incognito.teamcode.config.WorldSlideConstants.HS_CLAW_DROP_SPEED;
import static incognito.teamcode.config.WorldSlideConstants.HS_CLAW_GRAB_SPEED;
import static incognito.teamcode.config.WorldSlideConstants.HS_SLIDE_AUTO_OUT_POSITION;
import static incognito.teamcode.config.WorldSlideConstants.HS_DS_CONE_DISTANCE_CM;
import static incognito.teamcode.config.WorldSlideConstants.HS_DS_CONE_SUPER_DISTANCE_CM;
import static incognito.teamcode.config.WorldSlideConstants.HS_MAX_ENCODER;
import static incognito.teamcode.config.WorldSlideConstants.VS_CLAW_HANDOFF_SPEED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.Callable;
import java.util.concurrent.TimeUnit;

import incognito.cog.actions.Action;
import incognito.cog.actions.ActionManager;
import incognito.cog.opmodes.RobotLinearOpMode;
import incognito.cog.trajectory.TrajectorySequence;
import incognito.teamcode.robot.WorldRobot;
import incognito.teamcode.robot.component.arm.HorizontalArm;
import incognito.teamcode.robot.component.arm.VerticalArm;
import incognito.teamcode.robot.component.servoImplementations.Lever;

@Autonomous(name = "Ideal Auto Close Junction", group = "auto")
public class IdealAutoClose extends RobotLinearOpMode {
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

        Pose2d startPos = new Pose2d(35.5, -63.5, Math.toRadians(90));
        drivetrain.setPoseEstimate(startPos);

        double angle = 35;

        /*
        From starting position
        Positive X = to robot right
        Positive Y = to robot front
         */

        TrajectorySequence initialToJunction = drivetrain.trajectorySequenceBuilder(startPos) //r.startPose)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(
                        new Vector2d(20, -63.5),
                        Math.toRadians(180)
                )
                .splineToConstantHeading(
                        new Vector2d(10, -48),
                        Math.toRadians(90)
                )
                .addDisplacementMarker(() -> {
                    robot.horizontalArm.goToPosition(HorizontalArm.Position.UP);
                })
                .splineToSplineHeading(
                        new Pose2d(10, -12, Math.toRadians(180 + angle)),
                        Math.toRadians(90)
                )
                .splineToConstantHeading(
                        new Vector2d(11, -12),
                        Math.toRadians(angle)
                )
                .build();

        double firstYPref = -6;

        TrajectorySequence toStack = drivetrain.trajectorySequenceBuilder(initialToJunction.end())
                .addDisplacementMarker(() -> {
                    robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE);
                })
                .setTangent(Math.toRadians(angle))
                .splineToSplineHeading(
                        new Pose2d(20, -6, Math.toRadians(180)),
                        Math.toRadians(0)
                )
                .addDisplacementMarker(() -> {
                    robot.horizontalArm.goToPosition(HorizontalArm.Position.CLAW_OUT);
                    robot.horizontalArm.slide.setTargetPosition(HS_SLIDE_AUTO_OUT_POSITION);
                })
                .splineToConstantHeading(
                        new Vector2d(30, firstYPref),
                        Math.toRadians(0)
                )
                .splineToConstantHeading(
                        new Vector2d(40, firstYPref),
                        Math.toRadians(0)
                )
                /*.splineToConstantHeading(
                        new Vector2d(18, -18),
                        Math.toRadians(angle)
                )
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(
                        new Pose2d(26, -12, Math.toRadians(180)),
                        Math.toRadians(0)
                )
                .splineToSplineHeading(
                        new Pose2d(40, -12, Math.toRadians(180)),
                        Math.toRadians(0)
                )
                .waitSeconds(.25)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(
                        new Pose2d(24, -12, Math.toRadians(180)),
                        Math.toRadians(180)
                )
                .splineToSplineHeading(
                        new Pose2d(18, -18, Math.toRadians(180+angle)),
                        Math.toRadians(270)
                )*/
                .build();

        double xPref = 15;
        double deltaX = xPref - 11;
        double yPref = -9;
        double deltaY = yPref + 12;

        TrajectorySequence toJunction = drivetrain.trajectorySequenceBuilder(toStack.end())
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(
                        new Pose2d(20 + deltaX, -9, Math.toRadians(180 + angle)),
                        Math.toRadians(180)
                )
                .splineToSplineHeading(
                        new Pose2d(16, -10, Math.toRadians(180 + angle)),
                        Math.toRadians(180 + angle)
                )
                .build();

        TrajectorySequence toStack2 = drivetrain.trajectorySequenceBuilder(toJunction.end())
                .addDisplacementMarker(() -> {
                    robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE);
                })
                .setTangent(Math.toRadians(angle))
                .splineToSplineHeading(
                        new Pose2d(20 + deltaX, firstYPref, Math.toRadians(180)),
                        Math.toRadians(0)
                )
                .addDisplacementMarker(() -> {
                    robot.horizontalArm.goToPosition(HorizontalArm.Position.CLAW_OUT);
                    robot.horizontalArm.slide.setTargetPosition(HS_SLIDE_AUTO_OUT_POSITION);
                })
                .splineToConstantHeading(
                        new Vector2d(30 + deltaX, firstYPref),
                        Math.toRadians(0)
                )
                .splineToConstantHeading(
                        new Vector2d(40 + deltaX, firstYPref),
                        Math.toRadians(0)
                )
                .build();

        TrajectorySequence toJunction2 = drivetrain.trajectorySequenceBuilder(toStack.end())
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(
                        new Pose2d(20 + deltaX, -8, Math.toRadians(180 + angle)),
                        Math.toRadians(180)
                )
                .splineToSplineHeading(
                        new Pose2d(15, -10, Math.toRadians(180 + angle)),
                        Math.toRadians(180 + angle)
                )
                .build();

        /*TrajectorySequence toJunction2 = drivetrain.trajectorySequenceBuilder(toStack.end())
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(
                        new Pose2d(xPref, -10, Math.toRadians(180 + angle)),
                        Math.toRadians(180 + angle)
                )
                .build();*/

        Action highCone = new Action(robot.verticalArm::closeClaw)
                .then(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.WAIT_OUT))
                //.delay(0)
                .then(() -> robot.verticalArm.goToPosition(VerticalArm.Position.HIGH))
                .until(robot.verticalArm::atPosition)
                .then(robot.verticalArm::hingeDown)
                .delay(250)
                .then(robot.verticalArm::openClaw)
                .delay(100)
                //.then(() -> robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE))
                .globalize();

        Action intake = new Action(robot.horizontalArm::closeClaw)
                .delay(HS_CLAW_GRAB_SPEED)
                .then(() -> {
                    robot.verticalArm.openClaw();
                    robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE);
                    if (robot.horizontalArm.getPosition() != HorizontalArm.Position.IN)
                        robot.horizontalArm.goToPosition(HorizontalArm.Position.WAIT_IN);
                })
                .until(() -> robot.verticalArm.atPosition() && robot.horizontalArm.atPosition())
                .then(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.IN))
                .until(robot.horizontalArm::atPosition)
                .then(robot.horizontalArm::openClaw)
                .delay(HS_CLAW_DROP_SPEED)
                .then(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.UP))
                .delay(VS_CLAW_HANDOFF_SPEED)
                .then(robot.verticalArm::closeClaw)
                .globalize();

        waitForStart();
        robot.horizontalArm.slide.enable();

        robot.verticalArm.closeClaw();
        drivetrain.followTrajectorySequenceAsync(initialToJunction);
        nap(drivetrain::isBusy);

        highCone.run();
        nap(highCone::isActive);


        robot.horizontalArm.storeLeverHeight(Lever.HorizontalLeverPosition.FIFTH);
        drivetrain.followTrajectorySequenceAsync(toStack);
        nap(drivetrain::isBusy); //nap(() -> drivetrain.isBusy() || !robot.horizontalArm.claw.isClosed());

        robot.horizontalArm.goToPosition(HorizontalArm.Position.SUPER_OUT);
        nap(200);
        nap(robot.horizontalArm.claw::isClosed);
        nap(250);
        intake.run();

        //nap(1000);
        drivetrain.followTrajectorySequenceAsync(toJunction);
        nap(() -> intake.isActive() || drivetrain.isBusy());

        highCone.run();
        nap(highCone::isActive);

        robot.horizontalArm.storeLeverHeight(Lever.HorizontalLeverPosition.FOURTH);
        drivetrain.followTrajectorySequenceAsync(toStack2);
        nap(drivetrain::isBusy);

        robot.horizontalArm.goToPosition(HorizontalArm.Position.SUPER_OUT);
        nap(200);
        nap(robot.horizontalArm.claw::isClosed);
        nap(100);
        intake.run();

        drivetrain.followTrajectorySequenceAsync(toJunction2);
        nap(() -> intake.isActive() || drivetrain.isBusy());

        highCone.run();
        nap(highCone::isActive);

        robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE);
        nap(1000);
    }

    public void update() {
        ActionManager.update();
        robot.update();
        checkCancel();
        if (drivetrain.isBusy()) {
            drivetrain.update();
        }
    }

    public void checkCancel() {
        if (robot.horizontalArm.slide.getPosition() <= HS_MAX_ENCODER - 20) {
            return;
        }
        if (robot.horizontalArm.getDistance() > HS_DS_CONE_SUPER_DISTANCE_CM + 1) {
            stop();
        }

    }

    void nap(double milliseconds) {
        timer.reset();
        while (timer.time(TimeUnit.MILLISECONDS) <= milliseconds) {
            update();
        }
    }

    void nap(Callable<Boolean> condition) {
        try {
            while (condition.call()) {
                update();
                if (isStopRequested()) return;
            }
        } catch (Exception ignored) {}
    }
}