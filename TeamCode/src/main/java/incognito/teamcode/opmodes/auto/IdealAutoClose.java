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

        Pose2d startPos = new Pose2d(36, -62.5, Math.toRadians(90));
        drivetrain.setPoseEstimate(startPos);

        double angle = 15;

        TrajectorySequence firstGoToFirstJunction = drivetrain.trajectorySequenceBuilder(startPos) //r.startPose)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(
                        new Vector2d(12, -48),
                        Math.toRadians(90)
                )
                .splineToSplineHeading(
                        new Pose2d(14, -24, Math.toRadians(180 + angle)),
                        Math.toRadians(90)
                )
                .splineToConstantHeading(
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
                )
                .build();

        waitForStart();

        drivetrain.followTrajectorySequenceAsync(firstGoToFirstJunction);
        while (drivetrain.isBusy()) {
            update();
            if (isStopRequested()) break;
        }
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