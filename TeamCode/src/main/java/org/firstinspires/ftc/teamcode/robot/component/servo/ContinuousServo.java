package org.firstinspires.ftc.teamcode.robot.component.servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.component.HardwareComponent;

public class ContinuousServo extends HardwareComponent {
    private Servo servo;
    private double startPosition;
    private double lowerBound;
    private double upperBound;
    private double offsetFactor;

    public ContinuousServo(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo, double startPos) {
        this(hardwareMap, telemetry, crServo, startPos, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public ContinuousServo(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo,
                           double startPos, double lowBound, double upBound) {
        super(hardwareMap, telemetry);
        servo = crServo;
        startPosition = startPos;
        lowerBound = lowBound;
        upperBound = upBound;
        initializeHardware();
        goToStartPosition();
    }

    public Servo getServo() {
        return servo;
    }

    public void goToStartPosition() {
        servo.setPosition(startPosition);
    }

    public double getPosition() {
        return servo.getPosition();
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public void setOffsetFactor(double factor) {
        offsetFactor = factor;
    }

    public double getOffsetFactor() { return offsetFactor; }

    public void offsetPosition(double offset) {
        setPosition(getPosition() + offset * offsetFactor);
    }

    @Override
    protected void initializeHardware() {
        // do nothing
    }
}
