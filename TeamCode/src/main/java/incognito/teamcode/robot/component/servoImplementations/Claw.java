package incognito.teamcode.robot.component.servoImplementations;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import incognito.cog.hardware.component.servo.SetServo;

public class Claw extends SetServo {
    public Claw(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo,
                double openPos, double closedPos) {
        super(hardwareMap, telemetry, crServo, openPos, closedPos);
        close();
    }

    public Claw(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo,
                double openPos, double closedPos, double widePos) {
        super(hardwareMap, telemetry, crServo, new double[]{ openPos, closedPos, widePos });
        close();
    }

    public void open() {
        goToSetPosition(0);
    }

    public void close() {
        goToSetPosition(1);
    }

    public void wide() {
        goToSetPosition(2);
    }

    public void toggle() {
        if (isClosed()) {
            open();
        } else {
            close();
        }
    }

    public void toggleWide() {
        if (isClosed()) {
            wide();
        } else {
            close();
        }
    }

    public boolean isClosed() { return Math.abs(getPosition() - getSetPositionAtIndex(1)) < 0.01; }
    public boolean isOpened() { return !isClosed(); }
}