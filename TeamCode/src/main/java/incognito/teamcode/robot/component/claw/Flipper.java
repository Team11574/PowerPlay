package incognito.teamcode.robot.component.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import incognito.cog.hardware.component.servo.SetServo;

public class Flipper extends SetServo {
    public Flipper(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo,
                   double downPos, double upPos) {
        super(hardwareMap, telemetry, crServo, downPos, upPos);
    }

    public void flipDown() {
        goToSetPosition(0);
    }

    public void flipUp() {
        goToSetPosition(1);
    }
}