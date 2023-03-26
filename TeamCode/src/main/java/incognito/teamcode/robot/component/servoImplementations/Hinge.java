package incognito.teamcode.robot.component.servoImplementations;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Collections;

import incognito.cog.hardware.component.servo.ContinuousServo;

public class Hinge extends ContinuousServo {
    public enum Position {
        INTAKE,
        LOW_UP,
        LOW_DOWN,
        MEDIUM_UP,
        MEDIUM_DOWN,
        HIGH_UP,
        HIGH_DOWN,
    }

    public Hinge(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo, double startPos) {
        this(hardwareMap, telemetry, crServo, startPos, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public Hinge(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo, double[] startPositions) {
        this(hardwareMap, telemetry, crServo, startPositions, 0, 1);
    }

    public Hinge(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo,
                           double startPos, double lowBound, double upBound) {
        this(hardwareMap, telemetry, crServo, new double[]{startPos}, lowBound, upBound);
    }


    public Hinge(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo,
                           double[] startPositions, double lowBound, double upBound) {
        super(hardwareMap, telemetry, crServo, startPositions, lowBound, upBound);
    }

    public void goToSetPosition(Position position) {
        goToSetPosition(position, true);
    }

    public void goToSetPosition(Position position, boolean updateLastPosition) {
        goToSetPosition(position.ordinal(), updateLastPosition);
    }
}
