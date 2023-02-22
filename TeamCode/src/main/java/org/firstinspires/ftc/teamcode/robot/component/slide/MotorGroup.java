package org.firstinspires.ftc.teamcode.robot.component.slide;

import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.SET_POSITION_THRESHOLD;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.component.HardwareComponent;
import org.firstinspires.ftc.teamcode.robot.exceptions.UndefinedSetPositionException;

import java.util.ArrayList;

public class MotorGroup extends HardwareComponent {
    double MIN_ENCODER_POSITION;
    double MAX_ENCODER_POSITION;
    double TICKS_PER_INCH;
    double RUN_TO_POSITION_POWER;

    ArrayList<Integer> setPositions;
    double maxPower = 1;

    DcMotorEx[] motors;

    public MotorGroup(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx motor, double ticksPerInch) {
        this(hardwareMap, telemetry, new DcMotorEx[]{motor}, ticksPerInch);
    }
    public MotorGroup(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx motor, double minEncoderPosition, double maxEncoderPosition, double ticksPerInch) {
        this(hardwareMap, telemetry, new DcMotorEx[]{motor}, minEncoderPosition, maxEncoderPosition, ticksPerInch);
    }
    public MotorGroup(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx[] motors, double ticksPerInch) {
        this(hardwareMap, telemetry, motors, 0, Double.POSITIVE_INFINITY, ticksPerInch);
    }
    public MotorGroup(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx[] motors, double minEncoderPosition, double maxEncoderPosition, double ticksPerInch) {
        super(hardwareMap, telemetry);
        this.motors = motors;
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

    public void setMaxPower(double newPower) {
        maxPower = newPower;
    }

    public void setPower(double power) {
        if (power == 0 && motors[0].getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            return;
        }
        power = Math.min(power * maxPower, maxPower);
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
        RUN_TO_POSITION_POWER = Math.min(RUN_TO_POSITION_POWER * maxPower, maxPower);

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
        return totalCount / motors.length;
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