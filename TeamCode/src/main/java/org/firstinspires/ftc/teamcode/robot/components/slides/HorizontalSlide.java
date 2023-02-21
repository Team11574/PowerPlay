package org.firstinspires.ftc.teamcode.robot.components.slides;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class HorizontalSlide extends Slide {
    public HorizontalSlide(DcMotorEx slideMotor) {
        super(slideMotor);
    }

    public HorizontalSlide(DcMotorEx slideMotor, double maxHeight, double minEncoderPosition, double maxEncoderPosition) {
        super(slideMotor, maxHeight, minEncoderPosition, maxEncoderPosition);
    }

    public HorizontalSlide(DcMotorEx[] slideMotors) {
        super(slideMotors);
    }

    public HorizontalSlide(DcMotorEx[] slideMotors, double maxHeight, double minEncoderPosition,
            double maxEncoderPosition) {
        super(slideMotors, maxHeight, minEncoderPosition, maxEncoderPosition);
    }
}