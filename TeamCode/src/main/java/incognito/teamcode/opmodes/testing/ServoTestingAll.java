package incognito.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import incognito.cog.actions.Scheduler;
import incognito.cog.hardware.component.drive.Drivetrain;
import incognito.cog.hardware.component.servo.SetServo;
import incognito.teamcode.robot.WorldRobot;

@Disabled
@TeleOp(name = "Servo Testing All", group = "testing")
public class ServoTestingAll extends LinearOpMode {
    // Instance variables
    MultipleTelemetry multiTelemetry;
    Scheduler scheduler;
    WorldRobot robot;
    Drivetrain drivetrain;
    long waitTime = 500;
    long waitTimeLong = 1000;

    @Override
    public void runOpMode() {

        this.robot = new WorldRobot(hardwareMap, telemetry, false);
        this.drivetrain = robot.drivetrain;
        this.multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.scheduler = new Scheduler();

        log("===== INITIALIZING SERVO TESTS =====");

        log("<<< VERTICAL ARM SERVOS >>>");

        testServo(robot.verticalClaw, "verticalClaw");
        testServo(robot.verticalHinge, "verticalHinge");
        testServo(robot.verticalLever, "verticalLever");

        sleep(waitTimeLong);

        log("<<< HORIZONTAL ARM SERVOS >>>");

        testServo(robot.horizontalClaw, "horizontalClaw");
        testServo(robot.horizontalHinge, "horizontalHinge");
        testServo(robot.horizontalLever, "horizontalLever");


    }



    public void testServo(SetServo servo, String name) {
        testServo(servo, name, 0.0, 1.0);
    }

    public void testServo(SetServo servo, String name, double minPos, double maxPos) {
        log("Testing [" + name + "]");
        log("   <Full Range>");
        log("       Moving to 0.0");
        servo.setPosition(minPos);
        sleep(waitTime);
        log("       Moving to 0.5");
        servo.setPosition((maxPos - minPos) / 2 + minPos);
        sleep(waitTime);
        log("       Moving to 1.0");
        servo.setPosition(maxPos);
        sleep(waitTime);
        log("       Resetting to 0.0");
        servo.setPosition(minPos);
        sleep(waitTime);

        log("   <Set Positions>");
        for (int pos = 0; pos < servo.getSetPositionCount(); pos++) {
            log("       Moving to set position " + pos);
            if (pos < minPos || pos > maxPos) {
                log("           Position out of range, skipping");
                continue;
            }
            servo.setPosition(pos);
            sleep(waitTime);
        }

        log("       Resetting to 0.0");
        servo.setPosition(minPos);
        sleep(waitTime);

    }

    public void log(String message) {
        multiTelemetry.addLine(message);
        multiTelemetry.update();
    }
}