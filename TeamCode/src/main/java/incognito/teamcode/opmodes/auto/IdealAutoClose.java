package incognito.teamcode.opmodes.auto;

import static incognito.teamcode.config.WorldSlideConstants.HS_CLAW_DROP_SPEED;
import static incognito.teamcode.config.WorldSlideConstants.HS_CLAW_GRAB_SPEED;
import static incognito.teamcode.config.WorldSlideConstants.HS_SLIDE_AUTO_OUT_POSITION;
import static incognito.teamcode.config.WorldSlideConstants.HS_DS_CONE_SUPER_DISTANCE_CM;
import static incognito.teamcode.config.WorldSlideConstants.HS_SLIDE_MAX;
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

        double firstYPref = -6;

        Pose2d stackPose = new Pose2d(40, -7, Math.toRadians(180+0.5));

        TrajectorySequence toStack = drivetrain.trajectorySequenceBuilder(initialToJunction.end())
                .addDisplacementMarker(() -> {
                    robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE);
                })
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(
                        stackPose, //new Pose2d(stackPose.getX(), -6, stackPose.getHeading()), //new Pose2d(40, -12, Math.toRadians(180)),
                        Math.toRadians(0)
                )
                .addTemporalMarker(1, () -> {
                    robot.horizontalArm.goToPosition(HorizontalArm.Position.CLAW_OUT);
                    robot.horizontalArm.slide.setTargetPosition(HS_SLIDE_AUTO_OUT_POSITION);
                })
               /* .splineToConstantHeading(
                        new Vector2d(20, -10),
                        Math.toRadians(0)
                )
                .splineToSplineHeading(
                        new Pose2d(30, -6, Math.toRadians(180)),
                        Math.toRadians(0)
                )*/
                /*
                .splineToConstantHeading(
                        new Vector2d(30, firstYPref),
                        Math.toRadians(0)
                )*/
                /*.splineToConstantHeading(
                        new Vector2d(40, firstYPref),
                        Math.toRadians(0)
                )*/
                .build();

        double xPref = 15;
        double deltaX = xPref - 11;
        double yPref = -9;
        double deltaY = yPref + 12;

        TrajectorySequence toJunction = drivetrain.trajectorySequenceBuilder(toStack.end())
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(
                        new Vector2d(junctionPose.getX() + 10, junctionPose.getY()),
                        Math.toRadians(180)
                )
                .splineToSplineHeading(
                        new Pose2d(junctionPose.getX() + 1, -14, junctionPose.getHeading()),
                        Math.toRadians(180 - angle)
                )
                /*.splineToSplineHeading(
                        new Pose2d(30 + deltaX, -8, Math.toRadians(180 + angle)),
                        Math.toRadians(180)
                )
                .splineToSplineHeading(
                        new Pose2d(16, -10, Math.toRadians(180 + angle)),
                        Math.toRadians(180 + angle)
                )*/
                .build();

        TrajectorySequence toStack2 = drivetrain.trajectorySequenceBuilder(toJunction.end())
                .addDisplacementMarker(() -> {
                    robot.verticalArm.goToPosition(VerticalArm.Position.INTAKE);
                })
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(
                        new Pose2d(45, -8, Math.toRadians(180 - 1)), //new Pose2d(stackPose.getX(), -6, stackPose.getHeading()), //new Pose2d(40, -12, Math.toRadians(180)),
                        Math.toRadians(0)
                )
                .addTemporalMarker(1, () -> {
                    robot.horizontalArm.goToPosition(HorizontalArm.Position.CLAW_OUT);
                    robot.horizontalArm.slide.setTargetPosition(HS_SLIDE_AUTO_OUT_POSITION);
                })
                .build();

        TrajectorySequence toJunction2 = drivetrain.trajectorySequenceBuilder(toStack.end())
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(
                        new Vector2d(22, junctionPose.getY()),
                        Math.toRadians(180)
                )
                .splineToSplineHeading(
                        new Pose2d(16, -14, junctionPose.getHeading()),
                        Math.toRadians(180 - angle)
                )
                .build();

        TrajectorySequence spot1 = drivetrain.trajectorySequenceBuilder(toJunction2.end())
                .turn(Math.toRadians(-(angle + 90)))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(
                        new Vector2d(toJunction2.end().getX(), toJunction2.end().getY()-22),
                        Math.toRadians(270)
                )
                .build();

        TrajectorySequence spot2 = drivetrain.trajectorySequenceBuilder(toJunction2.end())
                .turn(Math.toRadians(-(angle + 90)))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(
                        new Vector2d(toJunction2.end().getX() + 1, toJunction2.end().getY() - 10),
                        Math.toRadians(270)
                )
                .splineToConstantHeading(
                        new Vector2d(toJunction2.end().getX() + 32.5, toJunction2.end().getY()-15),
                        Math.toRadians(0)
                )
                .build();

        TrajectorySequence spot3 = drivetrain.trajectorySequenceBuilder(toJunction2.end())
                .turn(Math.toRadians(-(angle + 90)))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(
                        new Vector2d(toJunction2.end().getX() + 1, toJunction2.end().getY() -25),
                        Math.toRadians(270)
                )
                .turn(Math.toRadians(-90))
                .setTangent(0)
                .splineToConstantHeading(
                        new Vector2d(toJunction2.end().getX()+49, toJunction2.end().getY()-25),
                        Math.toRadians(0)
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
                .delay(250)
                .then(robot.verticalArm::hingeDown)
                .delay(250)
                .then(robot.verticalArm::toggleClawWide)
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

        //nap(3000);

        telemetry.addLine("1");
        telemetry.update();
        robot.horizontalArm.storeLeverHeight(Lever.HorizontalLeverPosition.FIFTH);
        telemetry.addLine("2");
        telemetry.update();
        drivetrain.followTrajectorySequenceAsync(toStack);
        telemetry.addLine("3");
        telemetry.update();
        nap(drivetrain::isBusy); //nap(() -> drivetrain.isBusy() || !robot.horizontalArm.claw.isClosed());
        telemetry.addLine("4");
        telemetry.update();
        // ADD BACK
        robot.horizontalArm.goToPosition(HorizontalArm.Position.SUPER_OUT);
        telemetry.addLine("5");
        telemetry.update();

        nap(200);
        telemetry.addLine("6");
        telemetry.update();

        nap(robot.horizontalArm.claw::isClosed);
        robot.horizontalArm.slide.enable();
        telemetry.addLine("7");
        telemetry.update();

        robot.horizontalArm.closeClaw();
        nap(500);

        telemetry.addLine("8");
        telemetry.update();
        intake.run();
        telemetry.addLine("9");
        telemetry.update();

        //nap(1000);
        drivetrain.followTrajectorySequenceAsync(toJunction);
        telemetry.addLine("10");
        telemetry.update();
        nap(() -> intake.isActive() || drivetrain.isBusy());
        telemetry.addLine("11");
        telemetry.update();

        highCone.run();
        nap(highCone::isActive);
        // ADD BACK

        robot.horizontalArm.storeLeverHeight(Lever.HorizontalLeverPosition.FOURTH);
        drivetrain.followTrajectorySequenceAsync(toStack2);
        nap(drivetrain::isBusy);

        robot.horizontalArm.goToPosition(HorizontalArm.Position.SUPER_OUT);
        nap(200);

        nap(robot.horizontalArm.claw::isClosed);
        robot.horizontalArm.closeClaw();
        nap(500);
        intake.run();


        drivetrain.followTrajectorySequenceAsync(toJunction2);
        nap(() -> intake.isActive() || drivetrain.isBusy());

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
        if (robot.horizontalArm.slide.getPosition() <= HS_SLIDE_MAX - 10) {
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