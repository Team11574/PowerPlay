package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Component {
    // ===== Instance Variables =====

    // Inherit hardwareMap and telemetry from OpMode
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;

    public Component(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }
}
