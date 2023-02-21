package org.firstinspires.ftc.teamcode.robot.components.slides;

import static org.firstinspires.ftc.teamcode.robot.components.slides.SlideConstants.SET_POSITION_THRESHOLD;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.components.Component;
import org.firstinspires.ftc.teamcode.robot.components.HardwareComponent;
import org.firstinspires.ftc.teamcode.robot.exceptions.UndefinedSetPositionException;

import java.util.ArrayList;

public class Slide extends HardwareComponent {
    double MAX_POWER;
    double MIN_ENCODER_POSITION;
    double MAX_ENCODER_POSITION;
    double TICKS_PER_INCH;
    double RUN_TO_POSITION_POWER;

    ArrayList<Integer> setPositions;

    DcMotorEx[] motors;

    public Slide(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx slideMotor, double ticksPerInch) {
        this(hardwareMap, telemetry, new DcMotorEx[]{slideMotor}, ticksPerInch);
    }
    public Slide(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx slideMotor, double minEncoderPosition, double maxEncoderPosition, double ticksPerInch) {
        this(hardwareMap, telemetry, new DcMotorEx[]{slideMotor}, minEncoderPosition, maxEncoderPosition, ticksPerInch);
    }
    public Slide(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx[] slideMotors, double ticksPerInch) {
        this(hardwareMap, telemetry, slideMotors, 0, Double.POSITIVE_INFINITY, ticksPerInch);
    }
    public Slide(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx[] slideMotors, double minEncoderPosition, double maxEncoderPosition, double ticksPerInch) {
        super(hardwareMap, telemetry);
        this.motors = slideMotors;
        this.MIN_ENCODER_POSITION = minEncoderPosition;
        this.MAX_ENCODER_POSITION = maxEncoderPosition;
        this.TICKS_PER_INCH = ticksPerInch;

        initializeHardware();
    }

    protected void initializeHardware() {
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
    }

    public void setPower(double power) {
        if (power == 0 && motors[0].getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            return;
        }
        if (power > MAX_POWER)
            power = MAX_POWER;
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    public boolean atSetPosition() { return atSetPosition(SET_POSITION_THRESHOLD); }

    public boolean atSetPosition(double threshold) {
        double sum = 0;
        for (DcMotorEx motor : motors) {
            sum += motor.getCurrentPosition();
        }
        sum /= motors.length;
        return Math.abs(sum - motors[0].getTargetPosition()) <= threshold;
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

    public void goToSetPosition(int setPosition) {
        if (setPosition > setPositions.size()) {
            telemetry.addLine("Undefined set position!");
        }
        setTargetPosition(setPositions.get(setPosition));
    }

    public void hardReset() {
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void addSetPosition(int position) {
        setPositions.add(position);
    }

    public void addSetPositionLength(double length) {
        addSetPosition((int) (length * TICKS_PER_INCH));
    }

    public void addSetPositions(int[] positions) {
        for (int position : positions) {
            addSetPosition(position);
        }
    }

    public void addSetPositionLengths(double[] lengths) {
        for (double length : lengths) {
            addSetPositionLength(length);
        }
    }


    public void setSetPosition(int positionIndex, int positionValue) throws UndefinedSetPositionException {
        if (positionIndex > setPositions.size()) {
            throw new UndefinedSetPositionException();
        }
        setPositions.set(positionIndex, positionValue);
    }



    public void setSetPositionLength(int positionIndex, double positionValue) throws UndefinedSetPositionException {
        setSetPosition(positionIndex, (int) (positionValue * TICKS_PER_INCH));
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

    public double getVelocity() {
        double totalVel = 0;
        for (DcMotorEx motor : motors) {
            totalVel += motor.getVelocity();
        }
        return totalVel / motors.length;
    }

    public void stop() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }

}