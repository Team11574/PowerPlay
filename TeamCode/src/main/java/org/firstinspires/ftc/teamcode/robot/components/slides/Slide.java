package org.firstinspires.ftc.teamcode.robot.components.slides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.robot.exceptions.UndefinedSetPositionException;

import java.util.ArrayList;

class Slide {
    double MAX_LENGTH;
    double MIN_ENCODER_POSITION;
    double MAX_ENCODER_POSITION;

    ArrayList<Integer> setPositions;

    DcMotorEx[] motors;

    public Slide(DcMotorEx slideMotor) {
        this(new DcMotorEx[]{slideMotor});
    }
    public Slide(DcMotorEx slideMotor, double maxLength, double minEncoderPosition, double maxEncoderPosition) {
        this(new DcMotorEx[]{slideMotor}, maxLength, minEncoderPosition, maxEncoderPosition);
    }
    public Slide(DcMotorEx[] slideMotors) {
        this.motors = slideMotors;

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            PIDFCoefficients pid = new PIDFCoefficients(0,0,0,5);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        }
    }
    public Slide(DcMotorEx[] slideMotors, double maxHeight, double minEncoderPosition, double maxEncoderPosition) {
        this(slideMotors);
        this.MAX_LENGTH = maxHeight;
        this.MIN_ENCODER_POSITION = minEncoderPosition;
        this.MAX_ENCODER_POSITION = maxEncoderPosition;
    }

    public void setPower(double power) {
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
        }
    }

    public void setTargetPosition(int position) {
        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(position);
        }
    }

    public void goToSetPosition(int setPosition) throws UndefinedSetPositionException {
        if (setPosition > setPositions.size()) {
            throw new UndefinedSetPositionException();
        }
        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(setPositions.get(setPosition));
        }
    }

    public void cancelSetPosition() {
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void addSetPosition(int position) {
        setPositions.add(position);
    }

    public void addSetPositions(int[] positions) {
        for (int position : positions) {
            setPositions.add(position);
        }
    }

    public int getPosition() {
        int totalCount = 0;
        for (DcMotorEx motor : motors) {
            totalCount += motor.getCurrentPosition();
        }
        return totalCount/motors.length;
    }

    public void goToLength(double length) {
        this.setTargetPosition((int) (length/MAX_LENGTH * (MAX_ENCODER_POSITION - MIN_ENCODER_POSITION) + MIN_ENCODER_POSITION));
    }
}