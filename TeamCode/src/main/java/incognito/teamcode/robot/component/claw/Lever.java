package incognito.teamcode.robot.component.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import incognito.cog.hardware.component.servo.ContinuousServo;

public class Lever extends ContinuousServo {
    double discreteProgress = 0;
    double discreteAdvancementThreshold = 0.5;
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

    public void advancePositionDiscrete(double offset) {
        advancePositionDiscrete(offset, true);
    }

    // Offset position but only move to set positions
    // TODO: Test
    public void advancePositionDiscrete(double amount, boolean updateLastPosition) {
        discreteProgress += amount;
        if (Math.abs(discreteProgress) > discreteAdvancementThreshold) {
            shiftPositions((int) Math.signum(discreteProgress));
            discreteProgress = 0;
        }
    }

    public double getSetPositionAtIndex(LeverPosition pos) {
        return getSetPositionAtIndex(pos.ordinal());
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