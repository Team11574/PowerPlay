package incognito.teamcode.opmodes.auto;

import static incognito.teamcode.config.WorldSlideConstants.HS_CLAW_DROP_SPEED;
import static incognito.teamcode.config.WorldSlideConstants.HS_CLAW_GRAB_SPEED;
import static incognito.teamcode.config.WorldSlideConstants.HS_DS_CONE_SUPER_DISTANCE_CM;
import static incognito.teamcode.config.WorldSlideConstants.HS_MAX_ENCODER;
import static incognito.teamcode.config.WorldSlideConstants.HS_SLIDE_AUTO_OUT_POSITION;
import static incognito.teamcode.config.WorldSlideConstants.VS_CLAW_HANDOFF_SPEED;

import androidx.core.view.TintableBackgroundView;

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
import incognito.cog.trajectory.TrajectorySequenceBuilder;
import incognito.teamcode.robot.WorldRobot;
import incognito.teamcode.robot.component.arm.HorizontalArm;
import incognito.teamcode.robot.component.arm.VerticalArm;
import incognito.teamcode.robot.component.servoImplementations.Lever;

@Autonomous(name = "Ideal Auto One Cone", group = "auto")
public class IdealAutoOneCone extends RobotLinearOpMode {
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

        Pose2d junctionPose = new Pose2d(12, -13, Math.toRadians(180 + angle));

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
                        new Pose2d(9, -14, Math.toRadians(180 + angle)),
                        Math.toRadians(90)
                )
                .splineToConstantHeading(
                        new Vector2d(junctionPose.getX() - 1, junctionPose.getY() + 4),
                        Math.toRadians(angle)
                )
                .build();

        TrajectorySequence spot1 = drivetrain.trajectorySequenceBuilder(initialToJunction.end())
                .turn(Math.toRadians(-(angle + 90)))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(
                        new Vector2d(11, -31),
                        Math.toRadians(270)
                )
                .build();

        TrajectorySequence spot2 = drivetrain.trajectorySequenceBuilder(initialToJunction.end())
                .turn(Math.toRadians(-(angle + 90)))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(
                        new Vector2d(12, -19),
                        Math.toRadians(270)
                )
                .splineToConstantHeading(
                        new Vector2d(43.5, -24),
                        Math.toRadians(0)
                )
                .build();

        TrajectorySequence spot3 = drivetrain.trajectorySequenceBuilder(initialToJunction.end())
                .turn(Math.toRadians(-(angle + 90)))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(
                        new Vector2d(12, -34),
                        Math.toRadians(270)
                )
                .turn(Math.toRadians(-90))
                .setTangent(0)
                .splineToConstantHeading(
                        new Vector2d(60, -34),
                        Math.toRadians(0)
                )
                .build();




        Action highCone = new Action(robot.verticalArm::closeClaw)
                .then(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.WAIT_OUT))
                //.delay(0)
                .then(() -> robot.verticalArm.goToPosition(VerticalArm.Position.HIGH))
                .until(robot.verticalArm::atPosition)
                .delay(250)
                .then(robot.verticalArm::hingeDown)
                .delay(250)
                .then(robot.verticalArm::toggleClawWide)
                .delay(100)
                //.then(() -> robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE))
                .globalize();

        Action in = new Action(() -> {
                    robot.verticalArm.openClaw();
                    robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE);
                })
                .until(() -> robot.verticalArm.atPosition())
                .delay(250)
                .then(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.IN))
                .until(robot.horizontalArm::atPosition)
                .then(robot.horizontalArm::openClaw)
                .globalize();

        waitForStart();
        int parkingSpot = robot.getParkingSpot();
        robot.autoCamera.stopCamera();
        robot.horizontalArm.slide.enable();

        robot.verticalArm.closeClaw();
        drivetrain.followTrajectorySequenceAsync(initialToJunction);
        nap(drivetrain::isBusy);

        highCone.run();
        nap(highCone::isActive);

        in.run();

        switch (parkingSpot) {
            case 1:
                drivetrain.followTrajectorySequenceAsync(spot1);
                break;
            case 2:
                drivetrain.followTrajectorySequenceAsync(spot2);
                break;
            case 3:
                drivetrain.followTrajectorySequenceAsync(spot3);
                break;
        }

        nap(drivetrain::isBusy);
        nap(in::isActive);
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
        if (isStopRequested() || !opModeIsActive()) {
            terminateOpModeNow();
        }
        if (robot.horizontalArm.slide.getPosition() <= HS_MAX_ENCODER - 10) {
            return;
        }
        if (robot.horizontalArm.getDistance() > HS_DS_CONE_SUPER_DISTANCE_CM + 1) {
            terminateOpModeNow();
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
            }
        } catch (Exception ignored) {}
    }
}