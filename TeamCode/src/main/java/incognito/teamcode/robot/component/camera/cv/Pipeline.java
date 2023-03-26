package incognito.teamcode.robot.component.camera.cv;

import static incognito.teamcode.config.CameraConstants.aprilWeight;
import static incognito.teamcode.config.CameraConstants.colorWeight;
import static incognito.teamcode.config.CameraConstants.cx;
import static incognito.teamcode.config.CameraConstants.cy;
import static incognito.teamcode.config.CameraConstants.fx;
import static incognito.teamcode.config.CameraConstants.fy;
import static incognito.teamcode.config.CameraConstants.greenThreshold;
import static incognito.teamcode.config.CameraConstants.orangeThreshold;
import static incognito.teamcode.config.CameraConstants.purpleThreshold;
import static incognito.teamcode.config.CameraConstants.tagSize;
import static incognito.teamcode.config.CameraConstants.yellowLower;
import static incognito.teamcode.config.CameraConstants.yellowUpper;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Pipeline extends OpenCvPipeline {
    Telemetry telemetry;

    ArrayList<List<Scalar>> colorRanges;

    List<MatOfPoint> contours;
    MatOfPoint contour;
    Mat mask;
    MatOfPoint maxContour;

    Rect searchZone;

    Mat procFrame;
    Mat hierarchy;
    Mat val;
    Mat imageROI;
    ArrayList<Mat> channels;

    boolean stopped = false;

    ArrayList<Double> areas;

    List<Scalar> colorRange;

    int i, j, k;
    int maxIndex;

    double area;
    double area2;
    double maxArea;

    ArrayList<Integer> colorParkingList = new ArrayList<Integer>();
    ArrayList<Integer> aprilParkingList = new ArrayList<Integer>();

    // Number of frames to include in parking decision
    final int SAMPLES = 10;

    Mat yellowROI;
    Mat yellowMask;
    Mat valYellow;
    List<MatOfPoint> yellowContours;
    MatOfPoint yellowContour;
    ArrayList<Mat> yellowChannels;
    double yellowWidth;
    Moments moments;
    double yellowMaxWidth = 0;
    public double junctionHorizontalDistance = Double.POSITIVE_INFINITY;
    public double junctionWidth = 0;
    public double junctionHorizontalDistanceInternal = Double.POSITIVE_INFINITY;
    public boolean doingJunctions = false;
    double x;
    double y;
    DecimalFormat df = new DecimalFormat("0.00");

    long aprilDetector;
    Mat gray;
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    List<Integer> colorParkingSublist;
    List<Integer> aprilParkingSubList;

    int aprilParking;


    public Pipeline(Telemetry telem) {
        telemetry = telem;

        contours = new ArrayList<MatOfPoint>();
        mask = new Mat();
        maxContour = new MatOfPoint();

        yellowMask = new Mat();
        yellowContours = new ArrayList<>();
        yellowContour = new MatOfPoint();
        yellowChannels = new ArrayList<Mat>(3); // Channels for HSV image
        valYellow = new Mat();


        // Define list of color thresholds
        colorRanges = new ArrayList<List<Scalar>>(3);
        colorRanges.add(greenThreshold);
        colorRanges.add(orangeThreshold);
        colorRanges.add(purpleThreshold);

        // Extra hierarchy thing for contours
        hierarchy = new Mat();
        channels = new ArrayList<Mat>(3); // Channels for HSV image
        val = new Mat();

        // Zone to search for cones
        searchZone = new Rect(120, 0, 80, 240); // Creates a region in the middle of the frame to look

        areas = new ArrayList<Double>();

        gray = new Mat();

        aprilDetector = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
    public Mat processFrame(Mat input) {
        if (doingJunctions) {
            updateJunctionDistance(input);
            return input;
        } else {
            return processParking(input);
        }
    }


    public Mat processParking(Mat input) {
        if (stopped) {
            return input;
        }

        aprilParking = processAprilTags(input);

        telemetry.addData("April Parking", aprilParking);

        if (aprilParking != 0)
            aprilParkingList.add(aprilParking);

        if (imageROI != null)
            imageROI.release();

        imageROI = input.clone();

        // Creates a region of interest in the middle of the frame
        imageROI = imageROI.adjustROI(
                -searchZone.y,
                -(searchZone.height + searchZone.y) + 320,
                -searchZone.x,
                -(searchZone.width + searchZone.x) + 240
        );

        // Makes input to HSV from RGB image
        Imgproc.cvtColor(imageROI, imageROI, Imgproc.COLOR_RGB2HSV);

        // === Find largest area ===
        areas.clear();
        for (i = 0; i < colorRanges.size(); i++) {
            colorRange = colorRanges.get(i);

            area = getArea(imageROI, colorRange);
            areas.add(area);
        }

        maxArea = Collections.max(areas);
        maxIndex = areas.indexOf(maxArea);

        colorParkingList.add(maxIndex + 1);

        //telemetry.addData("Array", areas);
        //telemetry.addData("Orange Area", areas.get(0));
        //telemetry.addData("Purple Area", areas.get(1));
        //telemetry.addData("Green Area", areas.get(2));
        telemetry.addData("Max Area", maxArea);
        telemetry.addData("Zone", maxIndex + 1);
        telemetry.update();

        imageROI.release();
        return input;
    }

    public double getArea(Mat input, List<Scalar> colorRange) {
        procFrame = input.clone();

        maxArea = 0;

        Core.inRange(procFrame, colorRange.get(0), colorRange.get(1), mask);

        Core.split(procFrame, channels);

        Core.bitwise_and(channels.get(2), mask, val);

        contours.clear();

        Imgproc.findContours(val, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        for (j = 0; j < contours.size(); j++) {
            contour = contours.get(j);
            area2 = Imgproc.contourArea(contour);
            if (area2 > maxArea) {
                maxArea = area2;
            }
            contour.release();
        }

        procFrame.release();

        return maxArea;
    }

    /**
     * Gets the parking spot from the parking list
     * (using last frames, specified by SAMPLES)
     *
     * @return parking spot
     */
    public int getParkingSpot() {
        colorParkingSublist = colorParkingList;
        aprilParkingSubList = aprilParkingList;

        if (colorParkingList.size() > SAMPLES) {
            colorParkingSublist = colorParkingList.subList(Math.max(colorParkingList.size() - (SAMPLES + 1), 0), colorParkingList.size());
        }
        if (aprilParkingList.size() > SAMPLES) {
            aprilParkingSubList = aprilParkingList.subList(Math.max(colorParkingList.size() - (SAMPLES + 1), 0), colorParkingList.size());
        }
        int colorParkingValue = mode(colorParkingSublist);
        int aprilParkingValue = mode(aprilParkingSubList);
        int colorParkingCount = Collections.frequency(colorParkingSublist, colorParkingValue);
        int aprilParkingCount = Collections.frequency(aprilParkingSubList, aprilParkingValue);

        if (colorParkingCount * colorWeight > aprilParkingCount * aprilWeight) {
            return colorParkingValue;
        } else {
            return aprilParkingValue;
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
    public void start() {
        stopped = false;
    }

    public void updateJunctionDistance(Mat input) {
        // Use moments to find the center of yellow blobs

        telemetry.addLine("Junction distance update is running!");

        imageROI = input.clone();
        // TODO: Adjust ROI w/ imageROI.adjustROI()
        /*imageROI = imageROI.adjustROI(
                -150, 0, 0, 0
        );*/
        // Use moments to find the center of yellow blobs

        Imgproc.cvtColor(imageROI, imageROI, Imgproc.COLOR_RGB2HSV);

        Core.inRange(imageROI, yellowLower, yellowUpper, yellowMask);

        Core.split(imageROI, yellowChannels);

        Core.bitwise_and(yellowChannels.get(2), yellowMask, valYellow);

        Imgproc.findContours(valYellow, yellowContours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        imageROI.release();
        yellowMask.release();
        junctionHorizontalDistanceInternal = Double.POSITIVE_INFINITY;

        Imgproc.drawContours(input, yellowContours, -1, yellowUpper, 2);

        yellowMaxWidth = 0;

        for (k = 0; k < yellowContours.size(); k++) {
            yellowContour = yellowContours.get(k);
            // Create adjusted contour area to only take into account the width of the bounding rectangle
            yellowWidth = Imgproc.contourArea(yellowContour) / Imgproc.boundingRect(yellowContour).height;
            if (yellowWidth > yellowMaxWidth) {
                moments = Imgproc.moments(yellowContour, true);
                x = moments.get_m10() / moments.get_m00();
                y = moments.get_m01() / moments.get_m00();
                Imgproc.putText(input, df.format(yellowWidth), new Point(x-20, y), 5, 1, new Scalar(255, 255, 30));
                yellowMaxWidth = yellowWidth;
                telemetry.addData("Yellow max width", yellowMaxWidth);
                // why is this 170?
                junctionHorizontalDistanceInternal = 170 - x;
            }
            yellowContour.release();
        }

        junctionWidth = yellowMaxWidth;

        yellowContours.clear();
        junctionHorizontalDistance = junctionHorizontalDistanceInternal;
    }

    public int processAprilTags(Mat input) {
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
        /*gray.adjustROI(
                -searchZone.y,
                -(searchZone.height + searchZone.y) + 320,
                -searchZone.x,
                -(searchZone.width + searchZone.x) + 240);*/

        // Detect AprilTags
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(aprilDetector, gray, tagSize, fx, fy, cx, cy);

        for (AprilTagDetection detection : detections) {
            telemetry.addData("April tag detected", detection.id);
            return detection.id + 1;
        }

        gray.release();
        detections.clear();

        return 0;
    }
}