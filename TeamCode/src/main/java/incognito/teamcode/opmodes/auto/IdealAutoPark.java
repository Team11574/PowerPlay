package incognito.teamcode.opmodes.auto;

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

@Autonomous(name = "Ideal Auto Park", group = "auto")
public class IdealAutoPark extends RobotLinearOpMode {
    WorldRobot robot;
    MultipleTelemetry multiTelemetry;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() {
        this.robot = new WorldRobot(hardwareMap, telemetry, true);

        drivetrain = robot.drivetrain;

        ActionManager.clear();

        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.verticalArm.closeClaw();

        Pose2d startPos = new Pose2d(35.5, -63.5, Math.toRadians(90));

        drivetrain.setPoseEstimate(startPos);

        TrajectorySequence spot1 = drivetrain.trajectorySequenceBuilder(startPos)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(
                        new Vector2d(35.5, -58),
                        Math.toRadians(90)
                )
                .splineToSplineHeading(
                        new Pose2d(16, -50, Math.toRadians(180)),
                        Math.toRadians(180)
                )
                .build();

        TrajectorySequence spot2 = drivetrain.trajectorySequenceBuilder(startPos)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(
                        new Vector2d(35.5, -50),
                        Math.toRadians(90)
                )
                .build();

        TrajectorySequence spot3 = drivetrain.trajectorySequenceBuilder(startPos)
                .turn(-90)
                .forward(20)
                .build();


        waitForStart();
        int parkingSpot = robot.getParkingSpot();
        robot.autoCamera.stopCamera();
        robot.horizontalArm.slide.enable();
        robot.verticalArm.closeClaw();


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

    void nap(Callable<Boolean> condition) {
        try {
            while (condition.call()) {
                update();
            }
        } catch (Exception ignored) {}
    }
}