package org.firstinspires.ftc.teamcode.robot.components.slides;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.exceptions.UndefinedSetPositionException;

class Slide {
    double MAX_HEIGHT;
    double MAX_SPEED;
    double MIN_ENCODER_POSITION;
    double MAX_ENCODER_POSITION;

    int[] setPositions;

    DcMotor[] motors;


    public Slide(DcMotor slideMotor) {}
    public Slide(DcMotor slideMotor, double maxHeight, double minEncoderPosition, double maxEncoderPosition) {}
    public Slide(DcMotor[] slideMotors) {}
    public Slide(DcMotor[] slideMotors, double maxHeight, double minEncoderPosition, double maxEncoderPosition) {}

    public void setPower(double power) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

    public void setTargetPosition(int position) {
        for (DcMotor motor : motors) {
            motor.setTargetPosition(position);
        }
    }

    public void goToSetPosition(int setPosition) throws UndefinedSetPositionException {
        if (setPosition > setPositions.length) {
            throw new UndefinedSetPositionException();
        }
        for (DcMotor motor : motors) {
            motor.setTargetPosition(setPositions[setPosition]);
        }
    }

    public void cancelSetPosition() {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}