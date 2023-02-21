package org.firstinspires.ftc.teamcode.robot.components.slides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.robot.exceptions.UndefinedSetPositionException;

import java.util.ArrayList;

class Slide {
    double MAX_POWER;
    double MIN_ENCODER_POSITION;
    double MAX_ENCODER_POSITION;
    double TICKS_PER_INCH;
    double RUN_TO_POSITION_POWER;

    ArrayList<Integer> setPositions;

    DcMotorEx[] motors;

    public Slide(DcMotorEx slideMotor) {
        this(new DcMotorEx[]{slideMotor});
    }
    public Slide(DcMotorEx slideMotor, double minEncoderPosition, double maxEncoderPosition) {
        this(new DcMotorEx[]{slideMotor}, minEncoderPosition, maxEncoderPosition);
    }
    public Slide(DcMotorEx[] slideMotors) {
        this(slideMotors, 0, Double.POSITIVE_INFINITY);

    }
    public Slide(DcMotorEx[] slideMotors, double minEncoderPosition, double maxEncoderPosition) {
        this.motors = slideMotors;
        this.MIN_ENCODER_POSITION = minEncoderPosition;
        this.MAX_ENCODER_POSITION = maxEncoderPosition;

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            PIDFCoefficients pid = new PIDFCoefficients(0,0,0,5);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        }
    }

    public void setPower(double power) {
        for (DcMotorEx motor : motors) {
        if (power > MAX_POWER)
            power = MAX_POWER;
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setTargetPosition(int position) {
        // Run to position needs some power value to run at, default is 1
        RUN_TO_POSITION_POWER = Math.min(MAX_POWER, RUN_TO_POSITION_POWER);

        for (DcMotor motor : motors) {
            motor.setPower(RUN_TO_POSITION_POWER);
            motor.setTargetPosition(position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void goToSetPosition(int setPosition) throws UndefinedSetPositionException {
        if (setPosition > setPositions.size()) {
            throw new UndefinedSetPositionException();
        }
        setTargetPosition(setPositions.get(setPosition));
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
        setTargetPosition((int) (MIN_ENCODER_POSITION + TICKS_PER_INCH * length));
    }
}