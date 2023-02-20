package org.firstinspires.ftc.teamcode.robot.components.slides;

import com.qualcomm.robotcore.hardware.DcMotor;

public class VerticalSlide extends Slide {
    public VerticalSlide(DcMotor slideMotor) {
        super(slideMotor);
    }

    public VerticalSlide(DcMotor slideMotor, double maxHeight, double minEncoderPosition, double maxEncoderPosition) {
        super(slideMotor, maxHeight, minEncoderPosition, maxEncoderPosition);
    }

    public VerticalSlide(DcMotor[] slideMotors) {
        super(slideMotors);
    }

    public VerticalSlide(DcMotor[] slideMotors, double maxHeight, double minEncoderPosition,
            double maxEncoderPosition) {
        super(slideMotors, maxHeight, minEncoderPosition, maxEncoderPosition);
    }
}