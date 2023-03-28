package incognito.teamcode.robot.component.camera.cv;

import static incognito.teamcode.config.CameraConstants.BLUE_WIDTH_THRESHOLD;
import static incognito.teamcode.config.CameraConstants.RED_WIDTH_THRESHOLD;
import static incognito.teamcode.config.CameraConstants.VIEWPORT_WIDTH;
import static incognito.teamcode.config.CameraConstants.YELLOW_WIDTH_THRESHOLD;
import static incognito.teamcode.config.CameraConstants.aprilWeight;
import static incognito.teamcode.config.CameraConstants.blueLower;
import static incognito.teamcode.config.CameraConstants.blueThreshold;
import static incognito.teamcode.config.CameraConstants.blueUpper;
import static incognito.teamcode.config.CameraConstants.colorWeight;
import static incognito.teamcode.config.CameraConstants.cx;
import static incognito.teamcode.config.CameraConstants.cy;
import static incognito.teamcode.config.CameraConstants.fx;
import static incognito.teamcode.config.CameraConstants.fy;
import static incognito.teamcode.config.CameraConstants.greenThreshold;
import static incognito.teamcode.config.CameraConstants.orangeThreshold;
import static incognito.teamcode.config.CameraConstants.purpleThreshold;
import static incognito.teamcode.config.CameraConstants.redLower;
import static incognito.teamcode.config.CameraConstants.redThreshold;
import static incognito.teamcode.config.CameraConstants.redUpper;
import static incognito.teamcode.config.CameraConstants.tagSize;
import static incognito.teamcode.config.CameraConstants.yellowLower;
import static incognito.teamcode.config.CameraConstants.yellowThreshold;
import static incognito.teamcode.config.CameraConstants.yellowUpper;

import androidx.collection.ArraySet;

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

    int i, j, k, n, m;
    int maxIndex;

    double area;
    double area2;
    double maxArea;

    ArrayList<Integer> colorParkingList = new ArrayList<Integer>();
    ArrayList<Integer> aprilParkingList = new ArrayList<Integer>();

    // Number of frames to include in parking decision
    final int SAMPLES = 10;

    Mat junctionMask;
    Mat junctionV;
    ArrayList<List<Scalar>> junctionRanges;
    List<Scalar> junctionRange;
    List<MatOfPoint> junctionContours;
    MatOfPoint currentJunctionContour;
    ArrayList<Mat> junctionChannels;
    MatOfPoint maxJunctionContour;
    MatOfPoint junctionWidestContour;
    MatOfPoint junctionContourInternal;
    ArrayList<MatOfPoint> junctionWidestContours = new ArrayList<>();
    ArrayList<MatOfPoint> junctionContoursNonNull = new ArrayList<>();
    double currentWidth;
    Moments moments;
    double currentMaxWidth = 0;
    int currentMaxWidthIndex;
    Mat output;
    Scalar whiteScalar;
    Scalar blackScalar;
    public double junctionHorizontalDistance = Double.POSITIVE_INFINITY;
    public double junctionWidth = 0;
    public double junctionHorizontalDistanceInternal = Double.POSITIVE_INFINITY;
    public boolean doingJunctions = false;
    double x;
    double y;
    double redX = -1;
    double redY = -1;
    double blueX = -1;
    double blueY = -1;
    MatOfPoint yellowContour;
    double yellowWidth;
    MatOfPoint redContour;
    double redWidth;
    MatOfPoint blueContour;
    double blueWidth;
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

        junctionMask = new Mat();
        junctionContours = new ArrayList<>();
        currentJunctionContour = new MatOfPoint();
        junctionChannels = new ArrayList<Mat>(3); // Channels for HSV image
        junctionV = new Mat();
        output = new Mat();
        whiteScalar = new Scalar(0, 0, 255);
        blackScalar = new Scalar(0, 0, 0);


        // Define list of color thresholds
        colorRanges = new ArrayList<>(3);
        colorRanges.add(greenThreshold);
        colorRanges.add(orangeThreshold);
        colorRanges.add(purpleThreshold);

        // Define list of junction and cone thresholds
        junctionRanges = new ArrayList<>(3);
        junctionRanges.add(yellowThreshold);
        junctionRanges.add(redThreshold);
        junctionRanges.add(blueThreshold);

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
        //output.release();
        // Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2HSV);
        if (doingJunctions) {
            //updateJunctionDistanceYellow(input, output);
            //updateJunctionDistanceRed(input, output);
            //updateJunctionDistanceBlue(input, output);
            updateJunctionDistanceWithCones(input);
            //input.release();
            //Imgproc.cvtColor(output, output, Imgproc.COLOR_HSV2RGB);
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

    public MatOfPoint getWidestContour(Mat input, Scalar lower, Scalar upper) {
        // Use moments to find the center of yellow blobs
        Core.inRange(input, lower, upper, junctionMask);

        Core.split(input, junctionChannels);

        Core.bitwise_and(junctionChannels.get(2), junctionMask, junctionV);

        junctionContours.clear();

        Imgproc.findContours(junctionV, junctionContours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        junctionMask.release();
        junctionV.release();

        currentMaxWidth = 0;
        currentMaxWidthIndex = -1;

        for (k = 0; k < junctionContours.size(); k++) {
            currentJunctionContour = junctionContours.get(k);
            // Create adjusted contour area to only take into account the width of the bounding rectangle
            currentWidth = Imgproc.contourArea(currentJunctionContour) / Imgproc.boundingRect(currentJunctionContour).height;
            if (currentWidth > currentMaxWidth) {
                currentMaxWidthIndex = k;
                currentMaxWidth = currentWidth;
            }
        }
        if (currentMaxWidthIndex == -1) return null;
        for (k = 0; k < junctionContours.size(); k++) {
            if (k != currentMaxWidthIndex) {
                junctionContours.get(k).release();
            }
        }
        return junctionContours.get(currentMaxWidthIndex);
    }

    public void updateJunctionDistanceWithCones(Mat input) {
        // Make input into HSV
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        x = -1;
        y = -1;
        redX = -1;
        redY = -1;
        blueX = -1;
        blueY = -1;

        junctionWidestContours.clear();
        junctionContoursNonNull.clear();


        for (n = 0; n < 3; n++) {
            maxJunctionContour = getWidestContour(input, junctionRanges.get(n).get(0), junctionRanges.get(n).get(1));
            junctionWidestContours.add(maxJunctionContour);
            if (maxJunctionContour != null) {
                junctionContoursNonNull.add(maxJunctionContour);
                Imgproc.drawContours(input, junctionContoursNonNull, 0, junctionRanges.get(n).get(1), 2);
                junctionContoursNonNull.clear();
            }
        }

        yellowContour = junctionWidestContours.get(0);
        if (yellowContour != null && !yellowContour.empty()) {
            // We have a yellow contour, see if its width is within a usable threshold
            yellowWidth = Imgproc.contourArea(yellowContour) / Imgproc.boundingRect(yellowContour).height;
            if (yellowWidth > YELLOW_WIDTH_THRESHOLD) {
                moments = Imgproc.moments(yellowContour, true);
                x = moments.get_m10() / moments.get_m00();
                y = moments.get_m01() / moments.get_m00();
                telemetry.addData("Yellow width used", yellowWidth);
            }
        }
        // If no yellow was found
        if (x == -1) {
            telemetry.addData("No yellow found in threshold! Finding widest", "Blue and Red");
            redContour = junctionWidestContours.get(1);
            if (redContour != null) {
                redWidth = Imgproc.contourArea(redContour) / Imgproc.boundingRect(redContour).height;
                if (redWidth > RED_WIDTH_THRESHOLD) {
                    moments = Imgproc.moments(redContour, true);
                    redX = moments.get_m10() / moments.get_m00();
                    redY = moments.get_m01() / moments.get_m00();
                    telemetry.addData("Red width", redWidth);
                    telemetry.addData("Red x", redX);
                } else {
                    yellowWidth = -1;
                }
            }
            blueContour = junctionWidestContours.get(2);
            if (blueContour != null) {
                blueWidth = Imgproc.contourArea(blueContour) / Imgproc.boundingRect(blueContour).height;
                if (blueWidth > BLUE_WIDTH_THRESHOLD) {
                    moments = Imgproc.moments(blueContour, true);
                    blueX = moments.get_m10() / moments.get_m00();
                    blueY = moments.get_m01() / moments.get_m00();
                    telemetry.addData("Blue width", blueWidth);
                    telemetry.addData("Blue x", blueX);
                } else {
                    blueWidth = -1;
                }
            }
            if (redWidth > blueWidth) {
                telemetry.addData("Color used", "RED");
                x = redX;
                y = redY;
            } else if (blueWidth > redWidth) {
                telemetry.addData("Color used", "BLUE");
                x = blueX;
                y = blueY;
            } else {
                telemetry.addData("Color used", "NONE");
            }
        }

        for (m = 0; m < junctionWidestContours.size(); m++) {
            if (junctionWidestContours.get(m) != null)
                junctionWidestContours.get(m).release();
        }
        junctionWidestContours.clear();

        if (x < 0) {
            junctionHorizontalDistance = Double.POSITIVE_INFINITY;
        } else {
            junctionHorizontalDistance = VIEWPORT_WIDTH/2 - x;
            Imgproc.putText(input, df.format(junctionHorizontalDistance), new Point(x-20, y), 5, 1, whiteScalar);
            Imgproc.drawMarker(input, new Point(x, y), blackScalar);
        }
        telemetry.addData("Horizontal distance:", junctionHorizontalDistance);
        telemetry.update();

        Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2RGB);
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