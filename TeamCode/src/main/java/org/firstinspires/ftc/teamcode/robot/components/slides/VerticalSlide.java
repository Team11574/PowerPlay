package org.firstinspires.ftc.teamcode.robot.components.slides;

import com.qualcomm.robotcore.hardware.DcMotor;

public class VerticalSlide extends Slide {
    // TODO: Include max speed constructors
    public VerticalSlide(DcMotor slideMotor) {
        super(slideMotor);
    }

    public VerticalSlide(DcMotor slideMotor, double maxPower) {
        super(slideMotor, maxPower);
    }

    public VerticalSlide(DcMotor[] slideMotors) {
        super(slideMotors);
    }

    public VerticalSlide(DcMotor[] slideMotors, double maxPower) {
        super(slideMotors, maxPower);
    }

    public VerticalSlide(DcMotor slideMotor, double minEncoderPosition,
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