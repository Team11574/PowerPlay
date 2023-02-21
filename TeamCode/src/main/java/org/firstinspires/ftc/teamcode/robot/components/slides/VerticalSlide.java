package org.firstinspires.ftc.teamcode.robot.components.slides;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
        super(hardwareMap, telemetry, slideMotors, minEncoderPosition, maxEncoderPosition);
        this.motors = slideMotors;
        this.MIN_ENCODER_POSITION = minEncoderPosition;
        this.MAX_ENCODER_POSITION = maxEncoderPosition;

        initializeHardware();
    }
}