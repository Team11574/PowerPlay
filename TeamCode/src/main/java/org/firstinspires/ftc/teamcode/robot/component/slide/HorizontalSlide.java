package org.firstinspires.ftc.teamcode.robot.component.slide;

import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_BRAKE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_TICKS_PER_IN;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HorizontalSlide extends MotorGroup {

    public double stopDirection = 0;

    public HorizontalSlide(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx slideMotor) {
        this(hardwareMap, telemetry, new DcMotorEx[]{slideMotor});
    }

    public HorizontalSlide(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx slideMotor, double minEncoderPosition, double maxEncoderPosition) {
        this(hardwareMap, telemetry, new DcMotorEx[]{slideMotor}, minEncoderPosition, maxEncoderPosition);
    }

    public HorizontalSlide(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx[] slideMotors) {
        this(hardwareMap, telemetry, slideMotors, 0, Double.POSITIVE_INFINITY);
    }

    public HorizontalSlide(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx[] slideMotors, double minEncoderPosition, double maxEncoderPosition) {
        super(hardwareMap, telemetry, slideMotors, minEncoderPosition, maxEncoderPosition, HS_TICKS_PER_IN);
        this.motors = slideMotors;
        this.MIN_ENCODER_POSITION = minEncoderPosition;
        this.MAX_ENCODER_POSITION = maxEncoderPosition;
        //setTargetPosition(0);
        initializeHardware();
    }

    protected void initializeHardware() {
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            //motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, HS_PIDF);
        }
    }

    public void setPower(double power) {
        if (disabled)
            return;

        if (Math.abs(power) < 0.1 && motors[0].getMode() == DcMotorEx.RunMode.RUN_TO_POSITION) {
            return;
        }
        lastPower = power;
        if (stopDirection == 1 && power > 0) {
            power = 0;
        } else if (stopDirection == -1 && power < 0) {
            power = 0;
        } else {
            stopDirection = 0;
        }
        double realPower = Math.min(power * maxPower, maxPower);
        for (DcMotorEx motor : motors) {
            motor.setPower(realPower);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public void update() {
        super.update();
        if (getPosition() > MAX_ENCODER_POSITION - HS_BRAKE_THRESHOLD) {
            stopDirection = 1;
        } else if (getPosition() < MIN_ENCODER_POSITION + HS_BRAKE_THRESHOLD) {
            stopDirection = -1;
        }
    }
}