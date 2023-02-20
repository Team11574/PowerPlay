package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class HardwareComponent extends Component {
    public HardwareComponent(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        initializeHardware();
    }

    /**
     * Initializes component hardware.
     * i.e. Gets object from hardwareMap, sets motor run modes, etc.
     */
    protected abstract void initializeHardware();
}
