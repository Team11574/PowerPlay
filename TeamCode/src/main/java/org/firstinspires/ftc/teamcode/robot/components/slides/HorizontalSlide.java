package org.firstinspires.ftc.teamcode.robot.components.slides;

import com.qualcomm.robotcore.hardware.DcMotor;

public class HorizontalSlide extends Slide {
    // TODO: Include max speed constructors
    public HorizontalSlide(DcMotor slideMotor) {
        super(slideMotor);
    }

    public HorizontalSlide(DcMotor slideMotor, double maxHeight, double minEncoderPosition, double maxEncoderPosition) {
        super(slideMotor, maxHeight, minEncoderPosition, maxEncoderPosition);
    }

    public HorizontalSlide(DcMotor[] slideMotors) {
        super(slideMotors);
    }

    public HorizontalSlide(DcMotor[] slideMotors, double maxHeight, double minEncoderPosition,
            double maxEncoderPosition) {
        super(slideMotors, maxHeight, minEncoderPosition, maxEncoderPosition);
    }
}