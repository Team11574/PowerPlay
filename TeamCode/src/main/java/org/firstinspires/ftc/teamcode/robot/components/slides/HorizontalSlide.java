package org.firstinspires.ftc.teamcode.robot.components.slides;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HorizontalSlide extends Slide {
    public HorizontalSlide(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx slideMotor) {
        this(hardwareMap, telemetry, new DcMotorEx[]{slideMotor});
    }
    public HorizontalSlide(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx slideMotor, double minEncoderPosition, double maxEncoderPosition) {
        this(hardwareMap, telemetry, new DcMotorEx[]{slideMotor}, minEncoderPosition, maxEncoderPosition);
    }
    public HorizontalSlide(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx[] slideMotors) {
        this(hardwareMap, telemetry, slideMotors, 0, Double.POSITIVE_INFINITY);
    }
    public HorizontalSlide(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx[] slideMotors, double minEncoderPosition, double maxEncoderPosition) {
        super(hardwareMap, telemetry, slideMotors, minEncoderPosition, maxEncoderPosition);
        this.motors = slideMotors;
        this.MIN_ENCODER_POSITION = minEncoderPosition;
        this.MAX_ENCODER_POSITION = maxEncoderPosition;

        initializeHardware();
    }
}