package org.firstinspires.ftc.teamcode.robot.components.claws;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.components.HardwareComponent;

public class SetServo extends HardwareComponent {
    private Servo servo;
    private double[] positions;
    private int currentPositionIndex;

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
        positions = setPositions;
        servo = clawServo;
        goToSetPosition(0);
        initializeHardware();
    }

    public Servo getServo() {
        return servo;
    }

    public void goToSetPosition(int index) {
        if (index < 0) {
            index = positions.length - index;
        }
        if (index >= positions.length || index < 0) {
            telemetry.addLine("Servo undefined set position!");
        } else {
            servo.setPosition(positions[index]);
            currentPositionIndex = index;
        }
    }

    public int getCurrentPosition() {
        return currentPositionIndex;
    }

    public double getSetPositionAtIndex(int index) {
        if (index < 0) {
            index = positions.length - index;
        }
        if (index >= positions.length || index < 0) {
            telemetry.addLine("Servo undefined set position!");
            return -1;
        } else {
            return positions[index];
        }
    }


    @Override
    protected void initializeHardware() {
        // do nothing
    }
}