package incognito.cog.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import incognito.cog.component.drive.Drivetrain;
import incognito.teamcode.robot.Robot;

@Disabled
public class RobotOpMode extends OpMode {
    // Instance Variables
    protected Robot robot;
    protected Drivetrain drivetrain;

    @Override
    public void init() {
        this.robot = new Robot(hardwareMap, telemetry);
        this.drivetrain = robot.drivetrain;
    }

    @Override
    public void loop() {
        robot.update();
    }

}