package org.firstinspires.ftc.teamcode.robot.component.camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cog.component.HardwareComponent;
import org.openftc.easyopencv.OpenCvCamera;

public class CameraSuper extends HardwareComponent {

    // Instance Variables
    Pipeline pipeline;
    OpenCvCamera cvCamera;

    public CameraSuper(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    protected void initializeHardware() {

    }

    /**
     * Terminate Camera stream to save resources.
     */
    public void terminateCamera() {
        cvCamera.stopStreaming();
        pipeline.stop();
    }
}