package incognito.teamcode.robot.component.slide;

import static incognito.cog.util.Generic.withinThreshold;
import static incognito.teamcode.config.SlideConstants.S_SET_POSITION_THRESHOLD;
import static incognito.teamcode.config.SlideConstants.VS_ENCODER_CENTER;
import static incognito.teamcode.config.SlideConstants.VS_TICKS_PER_IN;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import incognito.cog.exceptions.UndefinedSetPositionException;

public class VerticalSlide extends MotorGroup {
    // Instance Variables
    DigitalChannel limitSwitch;

    public double stopDirection = 0;
    public boolean disabled = false;

    public VerticalSlide(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx slideMotor, DigitalChannel limitSwitch) {
        this(hardwareMap, telemetry, new DcMotorEx[]{slideMotor}, limitSwitch);
    }

    public VerticalSlide(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx slideMotor, double minEncoderPosition, double maxEncoderPosition, DigitalChannel limitSwitch) {
        this(hardwareMap, telemetry, new DcMotorEx[]{slideMotor}, minEncoderPosition, maxEncoderPosition, limitSwitch);
    }

    public VerticalSlide(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx[] slideMotors, DigitalChannel limitSwitch) {
        this(hardwareMap, telemetry, slideMotors, 0, Double.POSITIVE_INFINITY, limitSwitch);
    }

    public VerticalSlide(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx[] slideMotors, double minEncoderPosition, double maxEncoderPosition, DigitalChannel limitSwitch) {
        super(hardwareMap, telemetry, slideMotors, minEncoderPosition, maxEncoderPosition, VS_TICKS_PER_IN);
        this.motors = slideMotors;
        this.MIN_ENCODER_POSITION = minEncoderPosition;
        this.MAX_ENCODER_POSITION = maxEncoderPosition;
        this.limitSwitch = limitSwitch;
        setTargetPosition(0);
        initializeHardware();
    }

    protected void initializeHardware() {
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            //motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, VS_PIDF);
        }
    }

    public void goToSetPosition(SetPosition setPosition) {
        goToSetPosition(setPosition.ordinal());
    }

    // do we really need this
    public void setSetPosition(SetPosition setPosition, int positionValue) throws UndefinedSetPositionException {
        setSetPosition(setPosition.ordinal(), positionValue);
    }

    // do we really need this
    public void setSetPositionLength(SetPosition setPosition, double positionValue) throws UndefinedSetPositionException {
        setSetPositionLength(setPosition.ordinal(), positionValue);
    }

    public boolean getLimitState() {
        return !limitSwitch.getState();
    }

    public void setPower(double power) {
        if (disabled)
            return;
        if (Math.abs(power) < 0.1 && motors[0].getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            // In RUN_TO_POSITION MODE
            if (Math.signum(getTargetPosition() - getPosition()) == stopDirection && stopDirection != 0) {
                // If hit limit switch, actually set power to 0
                power = 0;
            } else {
                return;
            }
        }
        lastPower = power;
        if (stopDirection == 1 && power > 0) {
            power = 0;
        } else if (stopDirection == -1 && power < 0) {
            power = 0;
        } else {
            stopDirection = 0;
        }
        double realPower = Math.min(power * maxPower, maxPower);
        for (DcMotorEx motor : motors) {
            motor.setPower(realPower);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    public boolean goToTop() {
        update();
        setPower(maxPower);
        return stopDirection == 1;
    }

    public boolean goToBottom() {
        update();
        setPower(-maxPower);
        return stopDirection == -1;
    }

    @Override
    public boolean atSetPosition() {
        return atSetPosition(S_SET_POSITION_THRESHOLD);
    }

    @Override
    public boolean atSetPosition(double threshold) {
        /*
        if (motors[0].getTargetPosition() == 0 && stopDirection == -1) {
            setPower(0);
            return true;
        }
         */
        double sum = 0;
        for (DcMotorEx motor : motors) {
            sum += motor.getCurrentPosition();
        }
        sum /= motors.length;
        return withinThreshold(motors[0].getTargetPosition(), sum, threshold);
    }

    @Override
    public void update() {
        super.update();
        if (disabled) return;

        // If switch is pressed
        if (getLimitState()) {
            // If going upwards and at the top, stop
            if (getPosition() > VS_ENCODER_CENTER) {// && getDirection() > 0) {
                stopDirection = 1;
                // stop()
                // If going downwards and at the bottom, stop
            } else if (getPosition() <= VS_ENCODER_CENTER) { // && getDirection() < 0) {
                if (motors[0].getMode() == DcMotorEx.RunMode.RUN_USING_ENCODER) {
                    hardReset();
                } else if (motors[0].getMode() == DcMotor.RunMode.RUN_TO_POSITION &&
                        getPosition() > getTargetPosition()) {
                    hardReset();
                }
                stopDirection = -1;
            }
        }

        /*
        for (DcMotorEx motor : motors) {
            if (motor.isOverCurrent()) {
                disabled = true;
            }
        }

        if (disabled) disable_slide();
         */
    }

    /*
    public void disable_slide() {
        for (DcMotorEx motor : motors) {
            motor.setMotorDisable();
        }
    }
     */

    public enum SetPosition {
        GROUND,
        LOW,
        MEDIUM,
        HIGH,
        AUTO,
    }
}