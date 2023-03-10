package incognito.cog.component;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Component {
    // ===== Instance Variables =====

    // Inherit hardwareMap and telemetry from OpMode
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;

    public Component() {
    }

    public Component(HardwareMap hardwareMap, Telemetry telemetry) {
    }

}