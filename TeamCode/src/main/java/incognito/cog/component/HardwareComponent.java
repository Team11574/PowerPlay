package incognito.cog.component;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class HardwareComponent extends Component {
    public HardwareComponent(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    /**
     * Initializes component hardware.
     * i.e. Gets object from hardwareMap, sets motor run modes, etc.
     */
    protected abstract void initializeHardware();
}