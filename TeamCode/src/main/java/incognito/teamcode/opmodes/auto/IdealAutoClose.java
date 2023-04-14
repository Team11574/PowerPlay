package incognito.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        TrajectorySequence firstGoToFirstJunction = drivetrain.trajectorySequenceBuilder(startPos) //r.startPose)
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
                        new Vector2d(11, -10),
                        Math.toRadians(angle)
                )
                .build();
        TrajectorySequence toFirstStack = drivetrain.trajectorySequenceBuilder(firstGoToFirstJunction.end())
                .setTangent(0)
                .splineToSplineHeading(
                        new Pose2d(40, -12, Math.toRadians(180)),
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

        Action highCone = new Action(robot.verticalArm::closeClaw)
                .then(() -> robot.horizontalArm.goToPosition(HorizontalArm.Position.WAIT_OUT))
                //.delay(0)
                .then(() -> robot.verticalArm.goToPosition(VerticalArm.Position.HIGH))
                .until(robot.verticalArm::atPosition)
                .delay(50)
                .then(robot.verticalArm::openClaw)
                .delay(100)
                //.then(() -> robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE))
                .globalize();

        waitForStart();

        robot.verticalArm.closeClaw();
        drivetrain.followTrajectorySequenceAsync(firstGoToFirstJunction);
        while (drivetrain.isBusy()) {
            update();
            if (isStopRequested()) break;
        }
        highCone.run();
        while (highCone.isActive()) {
            update();
            if (isStopRequested()) break;
        }
        robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE);
        nap(1000);
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