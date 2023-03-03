package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.opmodes.base.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.component.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants;
import org.firstinspires.ftc.teamcode.robot.component.slide.VerticalSlide;
import org.firstinspires.ftc.teamcode.util.GamepadPlus;
import org.firstinspires.ftc.teamcode.util.runnable.Scheduler;

@TeleOp(name = "Motor Current Alert Testing", group = "testing")
public class MotorCurrentAlertTesting extends RobotOpMode {

    DcMotorEx testing_M;
    MultipleTelemetry tel;


    @Override
    public void init() {
        testing_M = hardwareMap.get(DcMotorEx.class, "testing_M");
        testing_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        tel.addData("Starting Current Alert", testing_M.getCurrentAlert(CurrentUnit.MILLIAMPS));
        tel.update();
    }

    @Override
    public void loop() {
        testing_M.setCurrentAlert(SlideConstants.CURRENT_ALERT, CurrentUnit.MILLIAMPS);
        testing_M.setPower(-gamepad1.left_stick_y);
        tel.addData("testing_M Current (mA)", testing_M.getCurrent(CurrentUnit.MILLIAMPS));
        tel.addData("testing_M Over Current", testing_M.isOverCurrent());
        tel.addData("testing_M Current Alert", testing_M.getCurrentAlert(CurrentUnit.MILLIAMPS));
        tel.update();
    }
}
