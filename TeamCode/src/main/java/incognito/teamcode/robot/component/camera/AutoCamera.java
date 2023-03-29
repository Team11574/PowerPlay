package incognito.teamcode.robot.component.camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import incognito.cog.hardware.component.camera.CameraSuper;
import incognito.teamcode.config.CameraConstants;
import incognito.teamcode.robot.component.camera.cv.Pipeline;

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
                //cvCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                cvCamera.startStreaming(CameraConstants.VIEWPORT_WIDTH, CameraConstants.VIEWPORT_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                startCamera();
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

    public void swapDoingJunctions() {
        pipeline.doingJunctions = !pipeline.doingJunctions;
    }

    public void setDoingJunctions(boolean b) {
        pipeline.doingJunctions = b;
    }

    // In pixels
    public double getJunctionDistance() {
        telemetry.addLine("AutoCamera --> getJunctionDistance");
        return pipeline.junctionHorizontalDistance;
    }

    public boolean coneOnJunction() {
        return pipeline.coneOnJunction();
    }

    // In pixels
    public double getJunctionWidth() {
        return pipeline.junctionWidth;
    }
}