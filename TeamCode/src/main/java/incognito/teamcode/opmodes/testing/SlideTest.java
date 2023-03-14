package incognito.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "slideTest", group = "testing")
public class SlideTest extends OpMode {

    DcMotorEx[] motors;
    DcMotorEx horizontal;

    int i;

    @Override
    public void init() {
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "VS_slideRight_M");
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "VS_slideLeft_M");
        horizontal = hardwareMap.get(DcMotorEx.class, "HS_slide_M");
        horizontal.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right.setDirection(DcMotorEx.Direction.REVERSE);
        motors = new DcMotorEx[]{right, left};
        i = 0;
        /*
        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(1000);
            motor.setPower(1);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

         */
    }

    @Override
    public void loop() {
        i++;
        if (i < 1000) {
            horizontal.setPower(0.5);
        } else {
            horizontal.setPower(0);
        }
        telemetry.addData("Position", horizontal.getCurrentPosition());
        telemetry.addData("Target Position", horizontal.getTargetPosition());
        telemetry.addData("Power", horizontal.getPower());
        telemetry.addData("Mode", horizontal.getMode());
        telemetry.update();

    }
}