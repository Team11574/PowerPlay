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
        MIN,
        MAX,
        INTAKE,
        LOW,
        MEDIUM,
        HIGH
    }


    // Instance Variables
    DigitalChannel limitSwitch;

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

    @Override
    public boolean getDangerState() {
        return !limitSwitch.getState();
    }
}