package incognito.teamcode.robot.component.servoImplementations;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import incognito.cog.hardware.component.servo.ContinuousServo;

public class Lever extends ContinuousServo {
    double discreteProgress = 0;
    double discreteAdvancementThreshold = 5;
    public Lever(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo,
                 double[] positions, double max, double min) {
        super(hardwareMap, telemetry, crServo, positions, max, min);
    }

    public void goToSetPosition(HorizontalLeverPosition pos) {
        goToSetPosition(pos, true);
    }

    public void goToSetPosition(HorizontalLeverPosition pos, boolean updateLastPosition) {
        goToSetPosition(pos.ordinal(), updateLastPosition);
    }

    public void goToSetPosition(VerticalLeverPosition pos) {
        goToSetPosition(pos, true);
    }

    public void goToSetPosition(VerticalLeverPosition pos, boolean updateLastPosition) {
        goToSetPosition(pos.ordinal(), updateLastPosition);
    }

    public void advancePositionDiscrete(double offset) {
        advancePositionDiscrete(offset, true);
    }

    // Offset position but only move to set positions
    // TODO: Test
    public void advancePositionDiscrete(double amount, boolean updateLastPosition) {
        if (amount == 0) {
            discreteProgress = discreteAdvancementThreshold;
            return;
        }
        discreteProgress += Math.abs(amount);
        if (Math.abs(discreteProgress) >= discreteAdvancementThreshold) {
            shiftPositions((int) Math.signum(amount));
            discreteProgress = 0;
        }
    }

    public double getSetPositionAtIndex(HorizontalLeverPosition pos) {
        return getSetPositionAtIndex(pos.ordinal());
    }

    public enum HorizontalLeverPosition {
        IN,
        MID,
        FIFTH,
        FOURTH,
        THIRD,
        SECOND,
        OUT,
    }

    public enum VerticalLeverPosition {
        INTAKE,
        LOW,
        MEDIUM,
        HIGH
    }

}