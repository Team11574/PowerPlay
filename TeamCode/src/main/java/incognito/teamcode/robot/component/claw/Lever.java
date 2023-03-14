package incognito.teamcode.robot.component.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import incognito.cog.hardware.component.servo.ContinuousServo;

public class Lever extends ContinuousServo {
    double stickyOffset = 0;
    double stickyOffsetThreshold = 0.1;
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

    public void offsetPositionSticky(double offset) {
        offsetPosition(offset, true);
    }

    // Offset position but only move to set positions
    // TODO: Test
    public void offsetPositionSticky(double offset, boolean updateLastPosition) {
        stickyOffset += offset;
        double stickyMax = getSetPositionCount() * stickyOffsetThreshold;
        if (stickyOffset < 0) {
            stickyOffset += stickyMax;
        } else if (stickyOffset > stickyMax) {
            stickyOffset -= stickyMax;
        }
        int index = (int) Math.round(stickyOffset / stickyOffsetThreshold);

        goToSetPosition(index, updateLastPosition);
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