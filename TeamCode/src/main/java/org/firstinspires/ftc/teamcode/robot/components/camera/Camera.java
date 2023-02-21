package org.firstinspires.ftc.teamcode.robot.components.camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.components.HardwareComponent;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Camera extends HardwareComponent {
    // Instance Variables
    Pipeline pipeline;
    OpenCvCamera cvCamera;

    public Camera(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        initializeHardware();
    }

    protected void initializeHardware(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        cvCamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new Pipeline(telemetry, true);

        cvCamera.setPipeline(pipeline);

        cvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                cvCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) { // This is if the camera doesn't work
                telemetry.addLine("Camera failed to initialize.");
            }
        });
    }

    /**
     * Terminate Camera stream to save resources.
     */
    public void terminateCamera(){
        cvCamera.stopStreaming();
        pipeline.stop();
    }

    /**
     * Get parking spot returned by pipeline
     * @return Parking spot, an integer 1-3
     */
    public int getParkingSpot() {
        return pipeline.getParkingSpot();
    }
}
