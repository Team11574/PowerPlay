package incognito.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import incognito.cog.actions.Scheduler;
import incognito.cog.opmodes.RobotLinearOpMode;
import incognito.teamcode.robot.Robot;

@Disabled
@TeleOp(name = "Front Arm Testing", group = "testing")
public class FrontArmTesting extends RobotLinearOpMode {
    // Instance variables
    MultipleTelemetry multiTelemetry;
    Scheduler scheduler;

    @Override
    public void runOpMode() {

        this.robot = new Robot(hardwareMap, telemetry, false);
        this.drivetrain = robot.drivetrain;
        this.multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.scheduler = new Scheduler();




        multiTelemetry.addLine("Yippee!");
        multiTelemetry.update();

    }
}