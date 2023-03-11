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
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class AutoPipeline extends OpenCvPipeline {
    Telemetry telemetry;

    Scalar orangeLow;
    Scalar orangeHigh;

    Scalar purpleLow;
    Scalar purpleHigh;

    Scalar greenLow;
    Scalar greenHigh;

    ArrayList<List<Scalar>> colorRanges;

    Mat output;
    Mat hierarchy;
    Mat val;
    ArrayList<Mat> channels;
    Rect searchZone;

    ArrayList<Integer> parkingList = new ArrayList<Integer>();

    boolean verbose;
    boolean stopped = false;

    List<MatOfPoint> contours;
    Mat mask;
    MatOfPoint maxContour;

    // Number of frames to include in parking decision
    final int SAMPLES = 10;

    // Default constructor
    public AutoPipeline(Telemetry telem) {
        this(telem, false);
    }

    // Constructor with verbose option
    public AutoPipeline(Telemetry telem, boolean verbose) {
        telemetry = telem; // Get telemetry object

        contours = new ArrayList<MatOfPoint>();
        mask = new Mat();
        maxContour = new MatOfPoint();

        // === Color Thresholds ===
        orangeLow = new Scalar(20, 40, 0); // Orange min HSV
        orangeHigh = new Scalar(30, 255, 255); // Orange max HSV

        purpleLow = new Scalar(130, 40, 0);
        purpleHigh = new Scalar(150, 255, 255);

        greenLow = new Scalar(40, 40, 0);
        greenHigh = new Scalar(70, 225, 225);

        // Extra hierarchy thing for contours
        hierarchy = new Mat();
        channels = new ArrayList<Mat>(3); // Channels for HSV image
        val = new Mat();

        // List of color ranges for easier iteration
        colorRanges = new ArrayList<List<Scalar>>();

        // Add color ranges to colorRanges
        colorRanges.add(Arrays.asList(orangeLow, orangeHigh));
        colorRanges.add(Arrays.asList(purpleLow, purpleHigh));
        colorRanges.add(Arrays.asList(greenLow, greenHigh));

        // Zone to search for cones
        searchZone = new Rect(180, 40, 140, 180); // Creates a region in the middle of the frame to look
        // for cone

        // Store verbose option
        this.verbose = verbose;
    }

    /**
     * Processes the input frame.
     * Finds area of each color and outputs the largest area.
     *
     * @param input
     * @return Telemetry output stream
     */
    @Override
    public Mat processFrame(Mat input) {
        if (stopped) {
            return input;
        }

        Mat procFrame = input.clone(); // Create a copy of the input frame for processing
        Imgproc.cvtColor(procFrame, procFrame, Imgproc.COLOR_RGB2HSV); // Makes input to HSV from
        // RGB image
        Mat imageROI = new Mat(procFrame, searchZone); // Creates a region of interest in the middle of the
        // frame

        // === Find largest area ===
        ArrayList<Double> areas = new ArrayList<Double>(); // List of areas for each color
        for (List<Scalar> colorRange : colorRanges) {
            Scalar low = colorRange.get(0);
            Scalar high = colorRange.get(1);

            double area = getArea(imageROI, low, high);
            areas.add(area);
        }

        // Find the largest area
        double maxArea = Collections.max(areas);
        int maxIndex = areas.indexOf(maxArea);

        // Add the max index to the parking list
        parkingList.add(maxIndex + 1);

        // === Output telemetry ===
        if (verbose) {
            telemetry.addData("Orange Area", areas.get(0));
            telemetry.addData("Purple Area", areas.get(1));
            telemetry.addData("Green Area", areas.get(2));
            telemetry.addData("Max Area", maxArea);
            telemetry.addData("Max Index+1", maxIndex + 1);
            telemetry.update();
        }

        // Release the processed frame
        procFrame.release();

        // Return the input frame
        return imageROI;
    }

    public double getArea(Mat input, Scalar low, Scalar high) {
        input = input.clone();
        double maxArea = 0;

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
        Imgproc.findContours(val, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);


        // Basic loop to find the contour with the largest area
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                maxContour = contour;
            }
        }

        return maxArea;
    }

    public double getArea_old(Mat input, Scalar low, Scalar high) {
        channels = new ArrayList<Mat>(3); // Channels for HSV image
        val = new Mat();
        Core.inRange(input, low, high, mask); // Creates a mask of the input frame with the color range
        // specified
        Core.split(input, channels);
        Core.bitwise_and(channels.get(2), mask, val);
        Imgproc.findContours(val, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        // Finds contours of the mask
        double maxArea = 0;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                maxContour = contour;
            }
        }
        mask.release();
        val.release();
        for (Mat chn : channels) {
            chn.release();
        }

        // Attempt to draw the contour
        // TODO: Fix this, currently doesn't work
        // since we don't do anything with the output frame.
        if (verbose)
            drawContour(input, maxContour, colorRanges.indexOf(Arrays.asList(low, high)));
        return maxArea;
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
    public int getParkingSpot() {
        if (parkingList.size() > SAMPLES) {
            return mode(parkingList.subList(Math.max(parkingList.size() - (SAMPLES + 1), 0), parkingList.size()));
        } else {
            return mode(parkingList);
        }
    }

    /**
     * Gets the mode of a list
     *
     * @param a - list to get mode of
     * @return mode
     */
    static int mode(List<Integer> a) {
        int output = 2, maxCount = 0;
        int i;
        int j;

        for (i = 0; i < a.size(); ++i) {
            int count = 0;
            for (j = 0; j < a.size(); ++j) {
                if (a.get(j) == a.get(i))
                    ++count;
            }

            if (count > maxCount) {
                maxCount = count;
                output = a.get(i);
            }
        }

        return output;
    }

    /**
     * Stops the pipeline
     */
    public void stop() {
        stopped = true;
    }

}