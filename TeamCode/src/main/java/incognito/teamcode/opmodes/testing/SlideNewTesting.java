package incognito.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import incognito.cog.actions.Scheduler;
import incognito.cog.opmodes.RobotLinearOpMode;
import incognito.teamcode.robot.Robot;
import incognito.teamcode.robot.component.slide.VerticalSlide;

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