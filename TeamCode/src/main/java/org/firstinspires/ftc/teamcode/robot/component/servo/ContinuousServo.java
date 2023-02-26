package org.firstinspires.ftc.teamcode.robot.component.servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.component.HardwareComponent;

public class ContinuousServo extends SetServo {
    //private Servo servo;
    private double startPosition;
    private double lowerBound;
    private double upperBound;
    private double offsetFactor = 1;

    public ContinuousServo(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo, double startPos) {
        this(hardwareMap, telemetry, crServo, startPos, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public ContinuousServo(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo, double[] startPositions) {
        this(hardwareMap, telemetry, crServo, startPositions, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public ContinuousServo(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo,
                           double startPos, double lowBound, double upBound) {
        this(hardwareMap, telemetry, crServo, new double[] {startPos}, lowBound, upBound);
    }
    
    

    public ContinuousServo(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo,
                           double[] startPositions, double lowBound, double upBound) {
        super(hardwareMap, telemetry, crServo, startPositions);
        startPosition = startPositions[0];
        telemetry.addData("Cont Servo Start Pos", startPosition);
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
        if (position < lowerBound) position = lowerBound;
        if (position > upperBound) position = upperBound;
        servo.setPosition(position);
    }

    public void setOffsetFactor(double factor) {
        offsetFactor = factor;
    }

    public double getOffsetFactor() { return offsetFactor; }

    public void offsetPosition(double offset) {
        setPosition(getPosition() + offset * offsetFactor);
    }

    /*
    TODO: For Dallin to fix later, add checking of set positions within bounds
    @Override
    public void addSetPositions(double[] newPositions) {
        double[] editedPositions = newPositions.clone();
        for (int i = 0; i < newPositions.length; i++) {
            if (newPositions[i] < lowerBound) {
                editedPositions[i] = lowerBound;
            } else if (newPositions[i] > upperBound) {
                editedPositions[i] = upperBound;
            }
        }
        super.addSetPositions(editedPositions);
    }

     */

    @Override
    protected void initializeHardware() {
        // do nothing
    }
}
