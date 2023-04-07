package incognito.teamcode.robot.component.servoImplementations;

import static incognito.teamcode.config.WorldSlideConstants.VS_HINGE_PAUSE_TIME;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import incognito.cog.hardware.component.servo.ContinuousServo;

public class HorizontalHinge extends ContinuousServo {
    public enum Position {
        IN,
        MID,
        FIFTH,
        FOURTH,
        THIRD,
        SECOND,
        OUT
    }

    public HorizontalHinge(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo, double startPos) {
        this(hardwareMap, telemetry, crServo, startPos, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public HorizontalHinge(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo, double[] startPositions) {
        this(hardwareMap, telemetry, crServo, startPositions, 0, 1);
    }

    public HorizontalHinge(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo,
                         double startPos, double lowBound, double upBound) {
        this(hardwareMap, telemetry, crServo, new double[]{startPos}, lowBound, upBound);
    }


    public HorizontalHinge(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo,
                         double[] startPositions, double lowBound, double upBound) {
        super(hardwareMap, telemetry, crServo, startPositions, lowBound, upBound);
    }

    public void goToSetPosition(Position HorizontalHingePosition) {
        goToSetPosition(HorizontalHingePosition.ordinal());
    }
}
