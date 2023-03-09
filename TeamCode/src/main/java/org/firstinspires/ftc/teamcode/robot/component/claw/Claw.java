package org.firstinspires.ftc.teamcode.robot.component.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cog.component.servo.SetServo;

public class Claw extends SetServo {
    public Claw(HardwareMap hardwareMap, Telemetry telemetry, Servo crServo,
                double openPos, double closedPos) {
        super(hardwareMap, telemetry, crServo, openPos, closedPos);
    }

    public void open() {
        goToSetPosition(0);
    }

    public void close() {
        goToSetPosition(1);
    }
}