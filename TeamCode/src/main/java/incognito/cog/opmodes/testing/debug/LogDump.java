package incognito.cog.opmodes.testing.debug;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import incognito.cog.opmodes.RobotLinearOpMode;

@Disabled
@Autonomous(name = "Log Dump", group = "auto")
public class LogDump extends RobotLinearOpMode {
    MultipleTelemetry tel;


    @Override
    public void runOpMode() {
    }
}