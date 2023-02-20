package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.components.camera.Pipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class RobotOLD {
    // ===== Instance Variables =====
    // -- Hardware --

    // Inherit hardwareMap and telemetry from OpMode
    HardwareMap hardwareMap;
    Telemetry telemetry;

    // Motor, Camera, IMU, etc. objects
    DcMotor flMotor = null;
    DcMotor frMotor = null;
    DcMotor blMotor = null;
    DcMotor brMotor = null;

    DcMotor[] motors = null;

    BHI260IMU imu = null;

    Pipeline pipeline;
    OpenCvCamera camera;

    // Motor Directions
    /**
     * Some motors are reversed to allow the mecanum drive to function properly.
     * Currently we are reversing the right motors which is kind of odd
     * when we reference the documentation, but I guess it's working?
     * https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#robot-centric-final-sample-code
     */
    DcMotorSimple.Direction flDirection = DcMotorSimple.Direction.FORWARD;
    DcMotorSimple.Direction frDirection = DcMotorSimple.Direction.REVERSE; // ** Reversed **
    DcMotorSimple.Direction blDirection = DcMotorSimple.Direction.FORWARD;
    DcMotorSimple.Direction brDirection = DcMotorSimple.Direction.REVERSE; // ** Reversed **


    // ===== Constants =====
    boolean VERBOSE;

    // Constants relating to wheel size and encoders
    final double WHEEL_RADIUS_MM = 48;
    final double WHEEL_RADIUS_IN = WHEEL_RADIUS_MM / 25.4;
    final double ENCODER_CPR = 280;

    // Calibration factors for inaccuracy in directional movement
    final double X_CAL = 5;
    final double Y_CAL = 3.7;

    // Threshold for when motors should start braking
    final double BRAKE_THRESH = 0.1;

    /**
     * Robot class to organize robot code.
     * @param hardwareMap hardwareMap Object
     * @param telemetry Telemetry object
     */
    public RobotOLD(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, false);
    }

    /**
     * Robot class to organize robot code.
     * @param hardwareMap hardwareMap Object
     * @param telemetry Telemetry object
     * @param verbose Verbose outputs for robot
     */
    public RobotOLD(HardwareMap hardwareMap, Telemetry telemetry, boolean verbose) {
        // Initialize hardwareMap and telemetry to external values
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        // Define verbosity
        this.VERBOSE = verbose;

        // Initialize robot hardware
        initialize_hardware();

        // Initialize camera
        initialize_camera();
    }

    /**
     * Retrieves hardware data from robot config and sets up motor values.
     */
    private void initialize_hardware() {
        flMotor = hardwareMap.get(DcMotor.class, "flMotor"); // Front Left motor
        frMotor = hardwareMap.get(DcMotor.class, "frMotor"); // Front Right motor
        blMotor = hardwareMap.get(DcMotor.class, "blMotor"); // Back Left motor
        brMotor = hardwareMap.get(DcMotor.class, "brMotor"); // Back Right motor

        motors = new DcMotor[]{flMotor, frMotor, blMotor, brMotor}; // List of all motors

        imu = hardwareMap.get(BHI260IMU.class, "imu"); // IMU

        configure_motors(); // Configure all motors
    }

    /**
     * Iterates through list of motors and configures each of them.
     */
    private void configure_motors() {
        for (DcMotor motor : motors) {
            configure_motor(motor);
        }
    }

    /**
     * Configures a motor by resetting encoders, setting to brake on zero power,
     * and then setting it to RUN_USING ENCODER.
     * @param motor The motor to configure
     */
    private void configure_motor(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoders
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Brake on zero power
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Run with encoders
    }

    /**
     * Initializes the camera with all of its necessary information.
     */
    private void initialize_camera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new Pipeline(telemetry, VERBOSE);


        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
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
    public void terminate_camera(){
        camera.stopStreaming();
        pipeline.stop();
    }

    /**
     * Get parking spot returned by pipeline
     * @return Parking spot, an integer 1-3
     */
    public int get_parking_spot() {
        return pipeline.getParkingSpot();
    }

    /**
     * Move forward distance with power.
     * @param distance Distance
     * @param power Power
     */
    public void forward(double distance, double power) {
        /*
            Wheel Diameter: 96 mm
            Wheel Radius: 48 mm
            Encoder Counts per Revolution: 280
            1 in = 25.4 mm
            ----------------------------------
            1 in => counts/280 * 2pi * radius
         */

        resetEncoders();

        double travelled_distance;
        do {
            travelled_distance = getAvgEncoderCount()/ENCODER_CPR * 2 * Math.PI * WHEEL_RADIUS_IN / Y_CAL;
            telemetry.addData("Travelled Distance", travelled_distance);
            telemetry.addData("Distance", distance);
            telemetry.addData("Distance > Travel", distance - Math.signum(distance) * BRAKE_THRESH > travelled_distance);
            telemetry.addData("Avg Encoder Count", getAvgEncoderCount());
            telemetry.update();
            move(0, power);
        } while(Math.abs(distance) - BRAKE_THRESH > travelled_distance);

        move(0, 0);
        telemetry.addData("Travelled Distance", travelled_distance);
        telemetry.addData("Distance", distance);
        telemetry.addData("Distance > Travel", distance  - Math.signum(distance) * BRAKE_THRESH > travelled_distance);
        telemetry.addData("Avg Encoder Count", getAvgEncoderCount());
        telemetry.update();
    }

    /**
     * Move back distance with power.
     * @param distance Distance
     * @param power Power
     */
    public void backward(double distance, double power) {
        forward(distance, -power);
    }

    /**
     * Move left distance with power.
     * @param distance Distance
     * @param power Power
     */
    public void left(double distance, double power) {
        resetEncoders();

        double travelled_distance;
        do {
            travelled_distance = getAvgEncoderCount()/ENCODER_CPR * 2 * Math.PI * WHEEL_RADIUS_IN / X_CAL;
            telemetry.addData("Travelled Distance", travelled_distance);
            telemetry.addData("Distance", distance);
            telemetry.addData("Avg Encoder Count", getAvgEncoderCount());
            move(-power, 0);
        } while(Math.abs(distance) - BRAKE_THRESH > travelled_distance);

        move(0, 0);
    }

    /**
     * Move right distance with power.
     * @param distance Distance
     * @param power Power
     */
    public void right(double distance, double power) {
        left(distance, -power);
    }

    /**
     * Reset all encoders
     */
    private void resetEncoders() {
        for (DcMotor motor : motors) {
            // Stop and reset
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Run again
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Get average value of all four encoders
     * @return Average encoder value
     */
    private int getAvgEncoderCount() {
        int flCount = Math.abs(flMotor.getCurrentPosition());
        int frCount = Math.abs(frMotor.getCurrentPosition());
        int blCount = Math.abs(blMotor.getCurrentPosition());
        int brCount = Math.abs(brMotor.getCurrentPosition());
        return (flCount + frCount + blCount + brCount)/4;

    }

    // Before: FL +, BL -, FR +, BR -
    // After: FL +, BL -, FR +, BR -

    /**
     * Move without rotating with specified x and y velocity
     * @param x X velocity
     * @param y Y velocity
     */
    public void move(double x, double y) {
        move(x, y, 0);
    }

    /**
     * Move with specified x and y velocities
     * and rotate in direction theta
     * @param velX X velocity
     * @param velY Y velocity
     * @param theta Direction
     */
    public void move(double velX, double velY, double theta) {
        double normalFactor = Math.max(Math.abs(velY) + Math.abs(velX) + Math.abs(theta), 1);
        double flPower = (velY + velX + theta) / normalFactor;
        double blPower = (velY - velX + theta) / normalFactor;
        double frPower = (velY - velX - theta) / normalFactor;
        double brPower = (velY + velX - theta) / normalFactor;
        blMotor.setPower(blPower);
        flMotor.setPower(flPower);
        brMotor.setPower(brPower);
        frMotor.setPower(frPower);
    }
}