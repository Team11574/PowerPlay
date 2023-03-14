package incognito.teamcode.robot.component.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import incognito.cog.hardware.component.servo.ContinuousServo;

public class Lever extends ContinuousServo {
    public Lever(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo,
                 double[] positions, double max, double min) {
        super(hardwareMap, telemetry, crServo, positions, max, min);
    }

    public void goToSetPosition(LeverPosition pos) {
        goToSetPosition(pos.ordinal());
    }

    public void goToSetPosition(LeverPosition pos, boolean updateLastPosition) {
        goToSetPosition(pos.ordinal(), updateLastPosition);
    }

    public enum LeverPosition {
        IN,
        MID,
        FIFTH,
        FOURTH,
        THIRD,
        SECOND,
        OUT,
    }

}