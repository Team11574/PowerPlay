package org.firstinspires.ftc.teamcode.robot.components.claws;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.components.HardwareComponent;

public class Claw extends HardwareComponent {
    public Claw(HardwareMap hardwareMap, Telemetry telemetry, CRServo servo) {
        super(hardwareMap, telemetry);
        initializeHardware();
    }

    protected void initializeHardware() {

    }
}
