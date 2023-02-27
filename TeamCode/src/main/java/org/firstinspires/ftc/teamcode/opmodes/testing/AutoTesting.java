package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.base.RobotLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.component.slide.VerticalSlide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.runnable.Scheduler;

@Autonomous(name = "AUTO Testing", group = "testing")
public class AutoTesting extends RobotLinearOpMode {
    // Instance variables
    MultipleTelemetry multiTelemetry;

    @Override
    public void runOpMode() {
        //super.runOpMode();
        this.robot = new Robot(hardwareMap, telemetry, true);

        drivetrain = robot.drivetrain;

        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        robot.lever.goToSetPosition(2); // 2 = out position
        robot.horizontalSlide.setTargetPosition(500);
        robot.horizontalClaw.open();
        robot.verticalClaw.open();

        sleep(2000);


        robot.horizontalClaw.close();
        sleep(500);
        robot.retractArm();
        robot.waitForRetract();
        robot.verticalClaw.close();
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.LOW);
        for (int i = 0; i < 200; i++) {
            multiTelemetry.addData("Vertical encoder", robot.verticalSlide.getPosition());
            multiTelemetry.update();
            sleep(10);
        }
        robot.depositCone();
        robot.waitForDeposit();
        robot.verticalSlide.goToSetPosition(VerticalSlide.SetPosition.GROUND);
        for (int i = 0; i < 100; i++) {
            multiTelemetry.addData("Vertical encoder", robot.verticalSlide.getPosition());
            multiTelemetry.update();
            sleep(10);
        }

        //drivetrain.followTrajectorySequence(backup);

        //robot.verticalSlide.goToSetPosition(0);

        multiTelemetry.addLine("Yippee!");
        multiTelemetry.update();

    }
}
