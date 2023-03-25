package incognito.teamcode.opmodes.testing;

import static incognito.teamcode.config.SlideConstants.CURRENT_ALERT_STOPPED;
import static incognito.teamcode.config.SlideConstants.VELOCITY_STOP_THRESHOLD;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import incognito.cog.hardware.component.servo.ContinuousServo;
import incognito.cog.hardware.gamepad.GamepadPlus;
import incognito.cog.opmodes.RobotOpMode;
import incognito.teamcode.config.SlideConstants;


@TeleOp(name = "Motor Current Alert Testing", group = "testing")
public class MotorCurrentAlertTesting extends RobotOpMode {

    DcMotorEx testing_M;
    MultipleTelemetry tel;
    ContinuousServo frontArm;
    Servo frontArmServo;

    GamepadPlus g;
    boolean pastOverCurrent;
    boolean pastOverStopCurrent;
    boolean disabled;

    DistanceSensor distanceSensor;

    @Override
    public void init() {
        testing_M = hardwareMap.get(DcMotorEx.class, "testing_M");
        testing_M.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        g = new GamepadPlus(gamepad1);
        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        tel.addData("Starting Current Alert", testing_M.getCurrentAlert(CurrentUnit.MILLIAMPS));

        frontArmServo = hardwareMap.get(Servo.class, "frontArm");
        frontArm = new ContinuousServo(hardwareMap, tel, frontArmServo, 0);
        frontArm.setPosition(0);

        tel.update();
    }

    @Override
    public void loop() {
        boolean overCurrent = testing_M.isOverCurrent();
        boolean overStopCurrent = (testing_M.getCurrent(CurrentUnit.MILLIAMPS) > CURRENT_ALERT_STOPPED &&
                testing_M.getVelocity() < VELOCITY_STOP_THRESHOLD);
        if (overCurrent || overStopCurrent) {
            testing_M.setMotorDisable();
            disabled = true;
        }

        if (g.x_pressed) {
            frontArm.setPosition(0);
        }
        if (g.y_pressed) {
            frontArm.setPosition(1);
        }

        if (overCurrent)
            pastOverCurrent = true;

        if (overStopCurrent)
            pastOverStopCurrent = true;

        tel.addData("OverCurrent", overCurrent);
        tel.addData("OverStopCurrent", overStopCurrent);
        tel.addData("HasOverCurrent", pastOverCurrent);
        tel.addData("HasOverStopCurrent", pastOverStopCurrent);

        if (g.a_pressed) {
            testing_M.setPower(0.75);
            testing_M.setTargetPosition(testing_M.getTargetPosition() + 10);
            testing_M.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (g.b_pressed) {
            testing_M.setPower(0.75);
            testing_M.setTargetPosition(testing_M.getTargetPosition() - 10);
            testing_M.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        testing_M.setCurrentAlert(SlideConstants.CURRENT_ALERT, CurrentUnit.MILLIAMPS);
        if (!disabled)
            testing_M.setPower(-gamepad1.left_stick_y);

        //tel.addData("Distance (cm)", distanceSensor.getDistance(DistanceUnit.CM));
        tel.addData("testing_M Current (mA)", testing_M.getCurrent(CurrentUnit.MILLIAMPS));
        tel.addData("testing_M Over Current", testing_M.isOverCurrent());
        tel.addData("testing_M Current Alert", testing_M.getCurrentAlert(CurrentUnit.MILLIAMPS));
        g.update();
        tel.update();
    }
}