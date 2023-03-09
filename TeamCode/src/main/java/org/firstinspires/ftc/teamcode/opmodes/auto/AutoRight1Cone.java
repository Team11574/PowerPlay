package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.cog.actions.Scheduler;
import org.firstinspires.ftc.teamcode.cog.opmodes.RobotLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.component.slide.VerticalSlide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AUTO Right 1 Cone", group = "auto", preselectTeleOp = "Tele")
public class AutoRight1Cone extends RobotLinearOpMode {
    // Instance variables
    TrajectorySequence spot1;
    TrajectorySequence spot2;
    TrajectorySequence spot3;

    TrajectorySequence[] spots;

    Scheduler scheduler;

    MultipleTelemetry tel;


    @Override
    public void runOpMode() {
        //super.runOpMode();
        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new Robot(hardwareMap, tel, true);

        drivetrain = robot.drivetrain;

        scheduler = new Scheduler();


        /*
        drive.trajectorySequenceBuilder(new Pose2d(36, -61.5, Math.toRadians(90)))
            .forward(51.5)
            .strafeLeft(12)
            .addDisplacementMarker(() -> {
                // Raise slides, flip claw, drop cone, etc
            })
            .back(2)
            //.strafeLeft(12)
            //.strafeRight(12)
            .strafeRight(33)
            .build()
         */

        Pose2d startPos = new Pose2d(0, 0, 0);

        drivetrain.setPoseEstimate(startPos);

        double deltaX = 0.5;

        TrajectorySequence cone = drivetrain.trajectorySequenceBuilder(startPos)
                //new Pose2d(36, -61.5, Math.toRadians(90)))
                .forward(56.5)
                .back(5.5)
                .strafeLeft(12 + deltaX)
                .build();

        /*
        TrajectorySequence backup = drivetrain.trajectorySequenceBuilder(cone.end())
                .back(2.5)
                .build();
         */

        spot1 = drivetrain.trajectorySequenceBuilder(cone.end())
                .strafeLeft(12 - deltaX)
                .back(21.5)
                .build();

        spot2 = drivetrain.trajectorySequenceBuilder(cone.end())
                .strafeRight(12 - deltaX)
                .back(21.5)
                .build();

        spot3 = drivetrain.trajectorySequenceBuilder(cone.end())
                .strafeRight(34.5 - deltaX)
                .back(21.5)
                .build();

        spots = new TrajectorySequence[]{spot1, spot2, spot3};

        waitForStart();
        robot.autoCamera.terminateCamera();

        int parkingSpot = robot.getParkingSpot();

        drivetrain.followTrajectorySequence(cone);

        runCone();

        /*

        //robot.verticalSlide.goToTop();

        while (!robot.verticalSlide.goToPositionConstant(VerticalSlide.SetPosition.AUTO)){ //!robot.verticalSlide.atSetPosition()) {
            telemetry.addData("Position", robot.verticalSlide.getPosition());
            telemetry.addData("Target", robot.verticalSlide.motors[0].getTargetPosition());
            telemetry.addData("Vel", robot.verticalSlide.motors[0].getVelocity());
            telemetry.addData("Power", robot.verticalSlide.motors[0].getPower());
            telemetry.addData("Stop dir", robot.verticalSlide.stopDirection);
            telemetry.addData("Mode", robot.verticalSlide.motors[0].getMode());
            telemetry.update();
        }
        robot.verticalSlide.setPower(VS_KG);

        /*
        flipping = true;
        opening = true;
        scheduler.linearSchedule(
                when -> true,
                then -> {
                    flipping = false;
                    robot.verticalClaw.open();
                },
                1000
        );
        scheduler.linearSchedule(
                when -> true,
                then -> {
                    opening = false;
                },
                1000
        );


        // At top, flip + open claw
        robot.verticalFlip.flipDown();

        while (flipping || opening) {
            scheduler.update();
        }
        robot.verticalClaw.close();
        robot.verticalFlip.flipUp();


        robot.depositCone();
        while (robot.verticalScheduler.hasLinearQueries()) {
            robot.update();
        }

        //drivetrain.followTrajectorySequence(backup);

        //robot.verticalSlide.goToSetPosition(0);


        while (!robot.verticalSlide.goToBottom()){ //!robot.verticalSlide.atSetPosition()) {
            telemetry.addData("Position", robot.verticalSlide.getPosition());
            telemetry.addData("Target", robot.verticalSlide.motors[0].getTargetPosition());
            telemetry.addData("Vel", robot.verticalSlide.motors[0].getVelocity());
            telemetry.addData("Power", robot.verticalSlide.motors[0].getPower());
            telemetry.addData("Stop dir", robot.verticalSlide.stopDirection);
            telemetry.addData("Mode", robot.verticalSlide.motors[0].getMode());
            telemetry.update();
        }
         */

        telemetry.addLine("Yippee!");
        telemetry.update();

        park(parkingSpot);

    }

    void runCone() {
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.HIGH);
        while (!robot.verticalSlide.atSetPosition()) {
            robot.update();
        }
        robot.depositCone();
        robot.waitForDeposit();
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.GROUND);
        while (!robot.verticalSlide.atSetPosition()) {
            robot.update();
        }
    }

    void park(int parkingSpot) {
        telemetry.addLine("Attempting to park in space " + parkingSpot);
        if (!isStopRequested()) {
            drivetrain.followTrajectorySequenceAsync(spots[parkingSpot - 1]);
            while (drivetrain.isBusy()) {
                drivetrain.update();
                robot.update();
            }
        }
    }
}