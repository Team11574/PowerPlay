package incognito.teamcode.robot.component.servoImplementations;

import static incognito.teamcode.config.WorldSlideConstants.VS_HINGE_PAUSE_TIME;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import incognito.cog.hardware.component.servo.ContinuousServo;

public class VerticalHinge extends ContinuousServo {
    public enum Position {
        INTAKE,
        LOW_UP,
        LOW_DOWN,
        MEDIUM_UP,
        MEDIUM_DOWN,
        HIGH_UP,
        HIGH_DOWN,
    }

    public Position storedPosition = Position.INTAKE;
    public double waitTime = 0;
    public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public VerticalHinge(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo, double startPos) {
        this(hardwareMap, telemetry, crServo, startPos, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public VerticalHinge(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo, double[] startPositions) {
        this(hardwareMap, telemetry, crServo, startPositions, 0, 1);
    }

    public VerticalHinge(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo,
                         double startPos, double lowBound, double upBound) {
        this(hardwareMap, telemetry, crServo, new double[]{startPos}, lowBound, upBound);
    }


    public VerticalHinge(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo,
                         double[] startPositions, double lowBound, double upBound) {
        super(hardwareMap, telemetry, crServo, startPositions, lowBound, upBound);
    }

    public void goToSetPosition(Position verticalHingePosition) {
        goToSetPosition(verticalHingePosition, true);
    }

    public void goToSetPosition(Position verticalHingePosition, boolean immediate) {
        goToSetPosition(verticalHingePosition, immediate, VS_HINGE_PAUSE_TIME);
    }

    public void goToSetPosition(Position verticalHingePosition, boolean immediate, double waitTime) {
        if (immediate) {
            goToSetPosition(verticalHingePosition.ordinal());
        } else {
            setPosition(lowerBound);
            storedPosition = verticalHingePosition;
            this.waitTime = waitTime;
            timer.reset();
        }
    }

    public void update() {
        if (waitTime != 0) {
            if (timer.time() > waitTime) {
                goToSetPosition(storedPosition.ordinal());
                this.waitTime = 0;
            }
        }
    }
}
