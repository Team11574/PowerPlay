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

    public void open() {
        goToSetPosition(0);
    }

    public void close() {
        goToSetPosition(1);
    }

    public boolean isClosed() { return getPosition() == getSetPositionAtIndex(1); }
    public boolean isOpened() { return getPosition() == getSetPositionAtIndex(0); }
}