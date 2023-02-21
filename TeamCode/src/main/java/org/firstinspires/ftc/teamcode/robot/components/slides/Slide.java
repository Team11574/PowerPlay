package org.firstinspires.ftc.teamcode.robot.components.slides;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.exceptions.UndefinedSetPositionException;

class Slide {
    double MAX_POWER;
    double MIN_ENCODER_POSITION;
    double MAX_ENCODER_POSITION;
    double run_to_position_power = 1;

    int[] setPositions;

    DcMotor[] motors;


    public Slide(DcMotor slideMotor) {
        this(new DcMotor[]{slideMotor});
    }

    public Slide(DcMotor slideMotor, double maxPower) {
        this(new DcMotor[]{slideMotor}, maxPower);
    }

    public Slide(DcMotor[] slideMotors) {
        this(slideMotors, 1);
    }

    public Slide(DcMotor[] slideMotors, double maxPower) {
        motors = slideMotors;
        MIN_ENCODER_POSITION = 0;
        MAX_ENCODER_POSITION = Double.POSITIVE_INFINITY;
        MAX_POWER = maxPower;
    }
    public Slide(DcMotor slideMotor, double minEncoderPosition, double maxEncoderPosition) {
        this(new DcMotor[]{slideMotor}, minEncoderPosition, maxEncoderPosition, 1);
    }

    public Slide(DcMotor slideMotor, double minEncoderPosition, double maxEncoderPosition, double maxPower) {
        this(new DcMotor[]{slideMotor}, minEncoderPosition, maxEncoderPosition, maxPower);
    }

    public Slide(DcMotor[] slideMotors, double minEncoderPosition, double maxEncoderPosition) {
        this(slideMotors, minEncoderPosition, maxEncoderPosition, 1);
    }

    public Slide(DcMotor[] slideMotors, double minEncoderPosition, double maxEncoderPosition, double maxPower) {
        motors = slideMotors;
        MIN_ENCODER_POSITION = minEncoderPosition;
        MAX_ENCODER_POSITION = maxEncoderPosition;
        MAX_POWER = maxPower;
    }

    public void setRunToPositionPower(double new_power) {
        if (new_power > MAX_POWER)
            new_power = MAX_POWER;
        run_to_position_power = new_power;

    }

    public void setPower(double power) {
        if (power > MAX_POWER)
            power = MAX_POWER;
        for (DcMotor motor : motors) {
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setTargetPosition(int position) {
        for (DcMotor motor : motors) {
            // Run to position needs some power value to run at, default is 1
            if (run_to_position_power > MAX_POWER) {
                run_to_position_power = MAX_POWER;
            }
            motor.setPower(run_to_position_power);
            motor.setTargetPosition(position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void goToSetPosition(int setPosition) throws UndefinedSetPositionException {
        if (setPosition > setPositions.length) {
            throw new UndefinedSetPositionException();
        }
        setTargetPosition(setPositions[setPosition]);
    }

    public void cancelSetPosition() {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}