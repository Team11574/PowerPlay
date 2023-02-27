package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.VS_KG;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.base.RobotLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.component.slide.VerticalSlide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.runnable.Scheduler;

@Autonomous(name = "AUTO Cone Left", group = "auto", preselectTeleOp = "Tele")
public class AutoConeLeft extends RobotLinearOpMode {
    // Instance variables
    TrajectorySequence spot1;
    TrajectorySequence spot2;
    TrajectorySequence spot3;

    TrajectorySequence[] spots;

    Scheduler scheduler;


    @Override
    public void runOpMode() {
        //super.runOpMode();
        this.robot = new Robot(hardwareMap, telemetry, true);

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

        Pose2d startPos = new Pose2d(0,0,0);

        drivetrain.setPoseEstimate(startPos);

        double deltaX = 0.5;

        TrajectorySequence cone = drivetrain.trajectorySequenceBuilder(startPos)
                //new Pose2d(36, -61.5, Math.toRadians(90)))
                .forward(56.5)
                .back(5.5)
                .strafeLeft(12+deltaX)
                .build();

        /*
        TrajectorySequence backup = drivetrain.trajectorySequenceBuilder(cone.end())
                .back(2.5)
                .build();
         */

        spot1 = drivetrain.trajectorySequenceBuilder(cone.end())
                .strafeLeft(12-deltaX)
                .back(21.5)
                .build();

        spot2 = drivetrain.trajectorySequenceBuilder(cone.end())
                .strafeRight(12-deltaX)
                .back(21.5)
                .build();

        spot3 = drivetrain.trajectorySequenceBuilder(cone.end())
                .strafeRight(34.5-deltaX)
                .back(21.5)
                .build();

        spots = new TrajectorySequence[]{spot1, spot2, spot3};

        waitForStart();

        int parkingSpot = robot.getParkingSpot();

        drivetrain.followTrajectorySequence(cone);

        //robot.verticalSlide.goToTop();

        while (!robot.verticalSlide.goToPositionConstant(VerticalSlide.SetPosition.AUTO)){ //!robot.verticalSlide.atSetPosition()) {
            telemetry.addData("Position", robot.verticalSlide.getPosition());
            telemetry.addData("Max power", robot.verticalSlide.maxPower);
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
         */

        robot.depositCone();
        while (robot.verticalScheduler.hasLinearQueries()) {
            robot.update();
        }

        //drivetrain.followTrajectorySequence(backup);

        //robot.verticalSlide.goToSetPosition(0);


        while (!robot.verticalSlide.goToBottom()){ //!robot.verticalSlide.atSetPosition()) {
            telemetry.addData("Position", robot.verticalSlide.getPosition());
            telemetry.addData("Max power", robot.verticalSlide.maxPower);
            telemetry.addData("Target", robot.verticalSlide.motors[0].getTargetPosition());
            telemetry.addData("Vel", robot.verticalSlide.motors[0].getVelocity());
            telemetry.addData("Power", robot.verticalSlide.motors[0].getPower());
            telemetry.addData("Stop dir", robot.verticalSlide.stopDirection);
            telemetry.addData("Mode", robot.verticalSlide.motors[0].getMode());
            telemetry.update();
        }

        telemetry.addLine("Yippee!");
        telemetry.update();

        park(parkingSpot);

    }

    void park(int parkingSpot) {
        telemetry.addLine("Attempting to park in space " + parkingSpot);
        if (!isStopRequested())
            drivetrain.followTrajectorySequence(spots[parkingSpot - 1]);

    }
}
