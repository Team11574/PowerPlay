package incognito.cog.component.camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import incognito.cog.component.camera.cv.Pipeline;

public class AutoCamera extends CameraSuper {

    public AutoCamera(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        initializeHardware();
    }

    @Override
    protected void initializeHardware() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        cvCamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new Pipeline(telemetry);//new AutoPipeline(telemetry, true);

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
     * Get parking spot returned by pipeline
     *
     * @return Parking spot, an integer 1-3
     */
    public int getParkingSpot() {
        return pipeline.getParkingSpot();
    }
}