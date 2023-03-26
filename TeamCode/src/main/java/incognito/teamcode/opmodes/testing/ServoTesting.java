package incognito.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import incognito.cog.actions.Scheduler;
import incognito.cog.hardware.component.drive.Drivetrain;
import incognito.cog.hardware.component.servo.SetServo;
import incognito.cog.hardware.gamepad.GamepadPlus;
import incognito.teamcode.robot.WorldRobot;

@TeleOp(name = "Servo Testing", group = "testing")
public class ServoTesting extends OpMode {
    // Instance variables
    MultipleTelemetry multiTelemetry;
    Scheduler scheduler;
    WorldRobot robot;
    Drivetrain drivetrain;
    long waitTime = 500;
    long waitTimeLong = 1000;
    GamepadPlus pad1;
    double moveSpeed = 0.01;
    int servoIndex = 0;
    SetServo[] servoList;

    @Override
    public void init() {
        this.robot = new WorldRobot(hardwareMap, telemetry, false);
        this.drivetrain = robot.drivetrain;
        this.multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.scheduler = new Scheduler();
        this.pad1 = new GamepadPlus(gamepad1);

        servoList = new SetServo[] {
                robot.verticalArm.claw,
                robot.verticalArm.hinge,
                robot.verticalArm.lever,
                robot.horizontalArm.claw,
                robot.horizontalArm.claw,
                robot.horizontalArm.claw
        };
        clearLog("Welcome to Servo Testing! Press start to begin.");
    }

    @Override
    public void loop() {
        SetServo currentServo = servoList[0];

        while (!pad1.a_pressed) {
            pad1.update();
            log("Select a servo, then press A to begin.");
            clearLog(">" + currentServo.toString());
            servoIndex += pad1.dpad_up_pressed ? 1 : (pad1.dpad_down_pressed ? -1 : 0);
            if (servoIndex < 0) {
                servoIndex += servoList.length;
            }
            currentServo = servoList[servoIndex];
        }

        log("Testing [" + currentServo.toString() + "]");
        testServo(currentServo, currentServo.toString());

    }

    public void testServo(SetServo servo, String name) {
        while (!pad1.b_pressed) {
            pad1.update();
            log("Testing [" + name + "]");
            servo.setPosition(servo.getPosition() + pad1.last_right_stick_y * moveSpeed);
            clearLog("Current servo position: " + servo.getPosition());
        }
    }

    public void log(String message) {
        multiTelemetry.addLine(message);
    }

    public void clearLog(String message) {
        multiTelemetry.addLine(message);
        multiTelemetry.update();
    }
}