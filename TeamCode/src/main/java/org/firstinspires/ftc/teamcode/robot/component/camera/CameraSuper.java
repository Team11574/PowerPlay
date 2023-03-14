package org.firstinspires.ftc.teamcode.robot.component.camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cog.component.HardwareComponent;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class CameraSuper extends HardwareComponent {

    // Instance Variables
    Pipeline pipeline;
    OpenCvCamera cvCamera;
    boolean isStreaming = true;

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
    public void stopCamera() {
        isStreaming = false;
        cvCamera.stopStreaming();
        pipeline.stop();
    }

    public void startCamera() {
        isStreaming = true;
        cvCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        pipeline.start();
    }

    public void toggleCamera() {
        if (isStreaming) {
            stopCamera();
        } else {
            startCamera();
        }
    }
}