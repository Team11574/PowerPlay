package org.firstinspires.ftc.teamcode.robot.component.slide;

import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_PIDF;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_TICKS_PER_IN;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.SET_POSITION_THRESHOLD;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HorizontalSlide extends MotorGroup {
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
        super(hardwareMap, telemetry, slideMotors, minEncoderPosition, maxEncoderPosition, HS_TICKS_PER_IN);
        this.motors = slideMotors;
        this.MIN_ENCODER_POSITION = minEncoderPosition;
        this.MAX_ENCODER_POSITION = maxEncoderPosition;
        goToSetPosition(0);
        initializeHardware();
    }

    protected void initializeHardware() {
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, HS_PIDF);
        }
    }

    public void update() {
        if (withinThreshold(getPosition(), MAX_ENCODER_POSITION, SET_POSITION_THRESHOLD)) {
            stop();
        } else if (withinThreshold(getPosition(), MIN_ENCODER_POSITION, SET_POSITION_THRESHOLD)) {
            stop();
        } else {
            refresh();
        }
    }
}