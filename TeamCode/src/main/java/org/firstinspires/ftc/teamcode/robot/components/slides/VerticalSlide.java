package org.firstinspires.ftc.teamcode.robot.components.slides;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class VerticalSlide extends Slide {
    public VerticalSlide(DcMotorEx slideMotor) {
    // TODO: Include max speed constructors
    public VerticalSlide(DcMotorEx slideMotor) {
        super(slideMotor);
    }

    public VerticalSlide(DcMotorEx slideMotor, double maxHeight, double minEncoderPosition, double maxEncoderPosition) {
        super(slideMotor, maxHeight, minEncoderPosition, maxEncoderPosition);
    public VerticalSlide(DcMotorEx slideMotor, double maxPower) {
        super(slideMotor, maxPower);
    }

    public VerticalSlide(DcMotorEx[] slideMotors) {
        super(slideMotors);
    }

    public VerticalSlide(DcMotorEx[] slideMotors, double maxHeight, double minEncoderPosition,
            double maxEncoderPosition) {
        super(slideMotors, maxHeight, minEncoderPosition, maxEncoderPosition);
    public VerticalSlide(DcMotorEx[] slideMotors, double maxPower) {
        super(slideMotors, maxPower);
    }

    public VerticalSlide(DcMotorEx slideMotor, double minEncoderPosition,
                         double maxEncoderPosition) {
        super(slideMotor, minEncoderPosition, maxEncoderPosition);
    }


    public VerticalSlide(DcMotor[] slideMotors, double minEncoderPosition,
                         double maxEncoderPosition) {
        super(slideMotors, minEncoderPosition, maxEncoderPosition);
    }

    public VerticalSlide(DcMotor slideMotor, double minEncoderPosition,
                         double maxEncoderPosition, double maxPower) {
        super(slideMotor, minEncoderPosition, maxEncoderPosition, maxPower);
    }


    public VerticalSlide(DcMotor[] slideMotors, double minEncoderPosition,
                         double maxEncoderPosition, double maxPower) {
        super(slideMotors, minEncoderPosition, maxEncoderPosition, maxPower);
    }
}