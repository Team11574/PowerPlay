package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.base.RobotLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.component.claw.Lever;
import org.firstinspires.ftc.teamcode.robot.component.slide.VerticalSlide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.runnable.Scheduler;

@Autonomous(name = "Slide New Testing", group = "testing")
public class SlideNewTesting extends RobotLinearOpMode {
    // Instance variables
    MultipleTelemetry multiTelemetry;
    Scheduler scheduler;

    @Override
    public void runOpMode() {
        this.robot = new Robot(hardwareMap, telemetry, true);

        drivetrain = robot.drivetrain;

        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        scheduler = new Scheduler();

        waitForStart();

        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.LOW);
        while (!robot.verticalSlide.atSetPosition()) {
            robot.update();
        }
        robot.depositCone();
        robot.waitForDeposit();
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.GROUND);
        while (!robot.verticalSlide.atSetPosition()) {
            robot.update();
        }

        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.MEDIUM);
        while (!robot.verticalSlide.atSetPosition()) {
            robot.update();
        }
        robot.depositCone();
        robot.waitForDeposit();
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.GROUND);
        while (!robot.verticalSlide.atSetPosition()) {
            robot.update();
        }

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
}
