package org.firstinspires.ftc.teamcode.robot.components.slides;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class VerticalSlide extends Slide {
    public VerticalSlide(DcMotorEx slideMotor) {
        super(slideMotor);
    }

    public VerticalSlide(DcMotorEx slideMotor, double maxHeight, double minEncoderPosition, double maxEncoderPosition) {
        super(slideMotor, maxHeight, minEncoderPosition, maxEncoderPosition);
    }

    public VerticalSlide(DcMotorEx[] slideMotors) {
        super(slideMotors);
    }

    public VerticalSlide(DcMotorEx[] slideMotors, double maxHeight, double minEncoderPosition,
            double maxEncoderPosition) {
        super(slideMotors, maxHeight, minEncoderPosition, maxEncoderPosition);
    }
}