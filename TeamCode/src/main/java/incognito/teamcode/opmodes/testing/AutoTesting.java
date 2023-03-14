package incognito.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import incognito.cog.actions.Scheduler;
import incognito.cog.opmodes.RobotLinearOpMode;
import incognito.teamcode.robot.Robot;
import incognito.teamcode.robot.component.claw.Lever;
import incognito.teamcode.robot.component.slide.VerticalSlide;

@Autonomous(name = "AUTO Testing", group = "testing")
public class AutoTesting extends RobotLinearOpMode {
    // Instance variables
    MultipleTelemetry multiTelemetry;
    Scheduler scheduler;

    @Override
    public void runOpMode() {
        //super.runOpMode();

        // drive.trajectorySequenceBuilder(new Pose2d(36, -61.5, Math.toRadians(90)))
        // Forward 57, left 2, rotate 160 degrees

        this.robot = new Robot(hardwareMap, telemetry, true);

        drivetrain = robot.drivetrain;

        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        scheduler = new Scheduler();

        waitForStart();
        robot.autoCamera.stopCamera();

        robot.lever.goToSetPosition(Lever.LeverPosition.FIFTH); // 5th cone height
        robot.levelHinge();
        robot.horizontalSlide.setTargetPosition(1640);
        robot.horizontalClaw.open();
        robot.verticalClaw.open();

        while (!robot.horizontalSlide.atSetPosition()) {
            // wait
            robot.update();
        }


        robot.horizontalClaw.close();
        sleep(250);
        robot.retractArm(false, true);
        robot.waitForRetract();
        sleep(350);
        robot.verticalClaw.close();
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.HIGH);
        scheduler.linearSchedule(
                when -> true,
                then -> {
                    robot.returnOut();
                    robot.lever.goToSetPosition(Lever.LeverPosition.FOURTH);
                    robot.levelHinge();
                },
                500
        );
        while (!robot.verticalSlide.atSetPosition()) {
            // wait
            robot.update();
            scheduler.update();
        }
        robot.depositCone();
        robot.waitForDeposit();
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.GROUND);
        robot.horizontalClaw.close();
        sleep(350);
        robot.retractArm(false, false);

        while (!robot.verticalSlide.atSetPosition()) {
            // wait
            robot.update();
        }
        robot.waitForRetract(); // double check horizontal is completed
        sleep(500);
        robot.horizontalClaw.open();
        sleep(250);
        robot.verticalClaw.close();
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.HIGH);
        while (!robot.verticalSlide.atSetPosition()) {
            // wait
            robot.update();
        }
        robot.depositCone();
        robot.waitForDeposit();
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.GROUND);
        while (!robot.verticalSlide.atSetPosition()) {
            // wait
            robot.update();
        }
        robot.horizontalSlide.goToSetPosition(0);
        robot.lever.goToSetPosition(Lever.LeverPosition.IN);
        while (!robot.horizontalSlide.atSetPosition()) {
            // wait
            robot.update();
        }


        //drivetrain.followTrajectorySequence(backup);

        //robot.verticalSlide.goToSetPosition(0);

        multiTelemetry.addLine("Yippee!");
        multiTelemetry.update();

    }
}