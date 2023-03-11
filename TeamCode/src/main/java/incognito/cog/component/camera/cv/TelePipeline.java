package incognito.cog.component.camera.cv;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class TelePipeline extends OpenCvPipeline {
    Telemetry telemetry;

    Scalar orangeLow;
    Scalar orangeHigh;

    Mat hierarchy;
    Mat val;
    ArrayList<Mat> channels;
    Rect searchZone;

    boolean verbose;
    boolean stopped = false;

    MatOfPoint contour;
    Mat mask;

    boolean nearPole;
    // TODO: Find actual minimum important pole contour area
    double MIN_POLE_AREA = 5000;

    // Default constructor
    public TelePipeline(Telemetry telem) {
        this(telem, false);
    }

    // Constructor with verbose option
    public TelePipeline(Telemetry telem, boolean verbose) {
        telemetry = telem; // Get telemetry object

        contour = new MatOfPoint();
        mask = new Mat();

        // === Color Thresholds ===
        orangeLow = new Scalar(20, 40, 0); // Orange min HSV
        orangeHigh = new Scalar(30, 255, 255); // Orange max HSV

        // Extra hierarchy thing for contours
        hierarchy = new Mat();
        channels = new ArrayList<Mat>(3); // Channels for HSV image
        val = new Mat();

        // Zone to search for poles
        // TODO: Find real rectangle too look for poles
        searchZone = new Rect(180, 40, 140, 180);

        // Store verbose option
        this.verbose = verbose;
    }

    /**
     * Processes the input frame.
     * Search for pole
     *
     * @param input
     * @return Telemetry output stream
     */
    @Override
    public Mat processFrame(Mat input) {
        nearPole = false;
        if (stopped) {
            return input;
        }

        Mat procFrame = input.clone(); // Create a copy of the input frame for processing
        Imgproc.cvtColor(procFrame, procFrame, Imgproc.COLOR_RGB2HSV); // Makes input to HSV from
        // RGB image
        Mat imageROI = new Mat(procFrame, searchZone); // Creates a region of interest in the middle of the
        // frame


        double area = getArea(imageROI, orangeLow, orangeHigh);

        if (area > MIN_POLE_AREA) {
            nearPole = true;
        }


        // === Output telemetry ===
        if (verbose) {
            telemetry.addData("Pole Area", area);
            telemetry.update();
        }

        // Release the processed frame
        procFrame.release();

        // Return the input frame
        return imageROI;
    }

    public double getArea(Mat input, Scalar low, Scalar high) {
        input = input.clone();

        Mat mask = new Mat();
        Core.inRange(input, low, high, mask); // Masks out the pixels that fit
        // within the
        // color range, so if a color is in the range, it becomes white (1) if not it is
        // black (0)
        Core.split(input, channels); // Separates the HSV image into separate Hue, Saturation, and Value maps

        //Mat hue;
        //Mat sat;
        //Mat val;
        //hue = new Mat(); // New matrix for each hue, saturation and value matrix
        //sat = new Mat();
        val = new Mat();

        //Core.bitwise_and(channels.get(0), mask, hue); // Apply the mask (thresh) found earlier to each channel
        //Core.bitwise_and(channels.get(1), mask, sat);// this was the only real way we could mask out the whole
        // image
        Core.bitwise_and(channels.get(2), mask, val);// while still preserving color

        /*
        ArrayList<Mat> chs = new ArrayList<Mat>();
        chs.add(hue);
        chs.add(sat);
        chs.add(val); // Adds all the hue, sat, and val matrices to a list so they can be combined

        Core.merge(chs, input); // Combines the masked out h,s,v matrices to make one big HSV image
        */
        ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();


        // We can only search for contours on a grayscale image, and we say that by
        // using the value channel
        // in an HSV image, we can achieve a similar effect for contour drawing
        //Imgproc.findContours(chs.get(2), contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Mat temp = new Mat();
        Imgproc.findContours(val, contours, temp, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // Basic loop to find the contour with the largest area
        double totalArea = 0;
        for (MatOfPoint contour : contours) {
            totalArea += Imgproc.contourArea(contour);
        }

        val.release();
        temp.release();
        mask.release();

        return totalArea;
    }

    /**
     * Draws a rectangle around the contours
     * for debugging purposes
     *
     * @param input      - input frame
     * @param maxContour - contour to draw
     * @param counter    - color counter
     * @return output frame
     */
    private Mat drawContour(Mat input, MatOfPoint maxContour, int counter) {
        Mat output = input.clone();
        Imgproc.rectangle(input, searchZone, new Scalar(0, 255, 0), 2);
        Rect bound = Imgproc.boundingRect(maxContour);
        Imgproc.rectangle(output, bound, new Scalar(counter * 100, counter * 100, 255 - counter * 100), 2);
        output.release();
        return output;
    }

    /**
     * Gets the parking spot from the parking list
     * (using last frames, specified by SAMPLES)
     *
     * @return parking spot
     */
    public boolean nearPole() {
        return nearPole;
    }

    /**
     * Stops the pipeline
     */
    public void stop() {
        stopped = true;
    }

    public void unstop() {
        stopped = false;
    }

    ;

}