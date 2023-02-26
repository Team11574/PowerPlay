package org.firstinspires.ftc.teamcode.robot.component.servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.component.HardwareComponent;

import java.util.ArrayList;
import java.util.List;

public class SetServo extends HardwareComponent {
    protected Servo servo;
    protected List<Double> positions;
    protected int currentPositionIndex;

    public SetServo(HardwareMap hardwareMap, Telemetry telemetry, Servo clawServo,
                    double position) {
        this(hardwareMap, telemetry, clawServo, new double[]{position});
    }

    public SetServo(HardwareMap hardwareMap, Telemetry telemetry, Servo clawServo,
                double firstPos, double secondPos) {
        this(hardwareMap, telemetry, clawServo, new double[]{firstPos, secondPos});
    }

    public SetServo(HardwareMap hardwareMap, Telemetry telemetry, Servo clawServo,
                    double[] setPositions) {
        super(hardwareMap, telemetry);
        positions = new ArrayList<>();
        addSetPositions(setPositions);
        servo = clawServo;
        goToSetPosition(0);
        initializeHardware();
    }

    public void addSetPositions(double[] newPositions) {
        for (double position : newPositions) {
            positions.add(position);
        }
    }

    public Servo getServo() {
        return servo;
    }

    public void goToSetPosition(int index) {
        if (index < 0) {
            index = positions.size() - index;
        }
        if (index >= positions.size() || index < 0) {
            telemetry.addLine("Servo undefined set position!");
        } else {
            servo.setPosition(positions.get(index));
            currentPositionIndex = index;
        }
    }

    public void toggle() {
        currentPositionIndex++;
        currentPositionIndex %= positions.size();
        goToSetPosition(currentPositionIndex);
    }

    public int getCurrentPositionIndex() {
        return currentPositionIndex;
    }

    public double getCurrentPosition() { return servo.getPosition(); }

    public double getSetPositionAtIndex(int index) {
        if (index < 0) {
            index = positions.size() - index;
        }
        if (index >= positions.size() || index < 0) {
            telemetry.addLine("Servo undefined set position!");
            return -1;
        } else {
            return positions.get(index);
        }
    }


    @Override
    protected void initializeHardware() {
        // do nothing
    }
}
