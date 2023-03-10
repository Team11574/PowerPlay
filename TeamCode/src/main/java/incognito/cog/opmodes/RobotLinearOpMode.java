package incognito.cog.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import incognito.cog.component.drive.Drivetrain;
import incognito.teamcode.robot.Robot;

@Disabled
public abstract class RobotLinearOpMode extends LinearOpMode {
    // Instance Variables
    protected Robot robot;
    protected Drivetrain drivetrain;

    @Override
    public void runOpMode() {
        this.robot = new Robot(hardwareMap, telemetry);
    }
}