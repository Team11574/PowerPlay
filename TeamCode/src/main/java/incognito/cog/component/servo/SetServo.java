package incognito.cog.component.servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

import incognito.cog.component.HardwareComponent;

public class SetServo extends HardwareComponent {
    protected Servo servo;
    protected List<Double> positions;
    protected int currentPositionIndex;
    protected double lastPosition;

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
        goToSetPosition(index, true);
    }

    public void goToSetPosition(int index, boolean updateLastPosition) {
        if (index < 0) {
            index = positions.size() - index;
        }
        if (index >= positions.size() || index < 0) {
            telemetry.addLine("Servo undefined set position!");
        } else {
            servo.setPosition(positions.get(index));
            if (updateLastPosition)
                lastPosition = positions.get(index);
            currentPositionIndex = index;
        }
    }

    public void goToLastPosition() {
        servo.setPosition(lastPosition);
    }

    public void toggle() {
        currentPositionIndex++;
        currentPositionIndex %= positions.size();
        goToSetPosition(currentPositionIndex);
    }

    public int getCurrentPositionIndex() {
        return currentPositionIndex;
    }

    public double getCurrentPosition() {
        return servo.getPosition();
    }

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