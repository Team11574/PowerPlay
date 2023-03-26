package incognito.teamcode.robot.component.slide;

import static incognito.cog.util.Generic.withinThreshold;
import static incognito.teamcode.config.WorldSlideConstants.VS_ENCODER_CENTER;
import static incognito.teamcode.config.WorldSlideConstants.VS_TICKS_PER_IN;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import incognito.cog.exceptions.UndefinedSetPositionException;
import incognito.cog.hardware.component.motor.MotorGroup;

public class VerticalSlide extends MotorGroup {

    public enum Position {
        INTAKE,
        LOW,
        MEDIUM,
        HIGH
    }


    // Instance Variables
    DigitalChannel limitSwitch;

    public boolean atTop = false;
    public boolean atBottom = true;

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
        initializeHardware();
        setTargetPosition(0);
    }

    public boolean goingUp() {
        if (isRunToPosition() && getTargetPosition() > getPosition()) return true;
        if (isRunUsingEncoder() && getPower() > 0) return true;
        return false;
    }

    public boolean goingDown() {
        if (isRunToPosition() && getTargetPosition() < getPosition()) return true;
        if (isRunUsingEncoder() && getPower() < 0) return true;
        return false;
    }

    protected void initializeHardware() {
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            //motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, VS_PIDF);
        }
    }

    public void goToSetPosition(Position setPosition) {
        goToSetPosition(setPosition.ordinal());
    }

    // do we really need this
    public void setSetPosition(Position position, int positionValue) throws UndefinedSetPositionException {
        setSetPosition(position.ordinal(), positionValue);
    }

    // do we really need this
    public void setSetPositionLength(Position position, double positionValue) throws UndefinedSetPositionException {
        setSetPositionLength(position.ordinal(), positionValue);
    }

    public boolean getLimitState() {
        return !limitSwitch.getState();
    }

    @Override
    public void setPower(double power) {
        if (atTop && goingUp())
            power = 0;
        else if (atBottom && goingDown())
            power = 0;
        lastPower = power;
        double realPower = Math.min(power * maxPower, maxPower);
        for (DcMotorEx motor : motors) {
            motor.setPower(realPower);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public boolean atSetPosition(double threshold) {
        // If limit switch is triggered and we are trying to go higher,
        // artificially say we are at our setPosition (highest value)
        if (atTop && goingUp()) {
            return true;
        }
        // If limit switch is triggered and we are trying to go lower,
        // artificially say we are at our setPosition (lowest value)
        if (atBottom && goingDown()) {
            return true;
        }
        double sum = 0;
        for (DcMotorEx motor : motors) {
            sum += motor.getCurrentPosition();
        }
        sum /= motors.length;
        return withinThreshold(motors[0].getTargetPosition(), sum, threshold);
    }

    @Override
    public void update() {
        //super.update();

        // If switch is pressed
        if (getLimitState()) {
            // If triggered above halfway point => at top
            if (getPosition() > VS_ENCODER_CENTER && goingUp()) {
                // Consider changing highest setPosition value to current position?
                // Don't reset if we are trying to go downwards
                atTop = true;
                setPower(0);
            // If triggered below halfway point => at bottom
            } else if (getPosition() <= VS_ENCODER_CENTER && goingDown()) {
                // Don't reset if we are trying to go upwards
                atBottom = true;
                setPower(0);
                hardReset();
            }
        } else {
            atBottom = false;
            atTop = false;
        }
    }
}