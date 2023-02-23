package org.firstinspires.ftc.teamcode.robot.component.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.component.servo.SetServo;

public class Lever extends SetServo {
    public Lever(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo,
                 double inPos, double midPos, double outPos) {
        super(hardwareMap, telemetry, crServo, new double[] {inPos, midPos, outPos} );
    }

    public void moveIn() {
        goToSetPosition(0);
    }

    public void moveMid() { goToSetPosition(1); }

    public void moveOut() {
        goToSetPosition(2);
    }


}
