package org.firstinspires.ftc.teamcode.robot.components.slides;

import static org.firstinspires.ftc.teamcode.robot.components.slides.SlideConstants.VS_PID;
import static org.firstinspires.ftc.teamcode.robot.components.slides.SlideConstants.VS_TICKS_PER_IN;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.exceptions.UndefinedSetPositionException;

public class VerticalSlide extends Slide {
    public VerticalSlide(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx slideMotor) {
        this(hardwareMap, telemetry, new DcMotorEx[]{slideMotor});
    }
    public VerticalSlide(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx slideMotor, double minEncoderPosition, double maxEncoderPosition) {
        this(hardwareMap, telemetry, new DcMotorEx[]{slideMotor}, minEncoderPosition, maxEncoderPosition);
    }
    public VerticalSlide(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx[] slideMotors) {
        this(hardwareMap, telemetry, slideMotors, 0, Double.POSITIVE_INFINITY);
    }
    public VerticalSlide(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx[] slideMotors, double minEncoderPosition, double maxEncoderPosition) {
        super(hardwareMap, telemetry, slideMotors, minEncoderPosition, maxEncoderPosition, VS_TICKS_PER_IN);
        this.motors = slideMotors;
        this.MIN_ENCODER_POSITION = minEncoderPosition;
        this.MAX_ENCODER_POSITION = maxEncoderPosition;

        initializeHardware();
    }

    protected void initializeHardware() {
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, VS_PID);
        }
    }

    public void goToSetPosition(SetPosition setPosition) throws UndefinedSetPositionException {
        goToSetPosition(setPosition.ordinal());
    }

    public void setSetPosition(SetPosition setPosition, int positionValue) throws UndefinedSetPositionException {
        setSetPosition(setPosition.ordinal(), positionValue);
    }

    public void setSetPositionLength(SetPosition setPosition, double positionValue) throws UndefinedSetPositionException {
        setSetPositionLength(setPosition.ordinal(), positionValue);
    }

    public enum SetPosition {
        LOW,
        MEDIUM,
        HIGH
    }
}