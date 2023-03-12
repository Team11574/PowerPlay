package org.firstinspires.ftc.teamcode.robot.component.camera;

import static org.firstinspires.ftc.teamcode.robot.component.camera.CameraConstants.greenThreshold;
import static org.firstinspires.ftc.teamcode.robot.component.camera.CameraConstants.orangeThreshold;
import static org.firstinspires.ftc.teamcode.robot.component.camera.CameraConstants.purpleThreshold;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

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

    ArrayList<Integer> parkingList = new ArrayList<Integer>();

    // Number of frames to include in parking decision
    final int SAMPLES = 10;

    Mat yellowROI;
    Mat yellowMask;
    Mat valYellow;
    List<MatOfPoint> yellowContours;
    MatOfPoint yellowContour;
    ArrayList<Mat> yellowChannels;
    Moments moments;
    double yellowContourArea = 0;
    public double junctionDistance = Double.POSITIVE_INFINITY;
    public double junctionArea = 0;
    public double junctionDistanceInternal = Double.POSITIVE_INFINITY;
    public boolean doingJunctions = false;
    double x;

    Scalar yellowLower = new Scalar(20, 150, 120);
    Scalar yellowUpper = new Scalar(40, 255, 255);


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
        colorRanges.add(orangeThreshold);
        colorRanges.add(purpleThreshold);
        colorRanges.add(greenThreshold);

        // Extra hierarchy thing for contours
        hierarchy = new Mat();
        channels = new ArrayList<Mat>(3); // Channels for HSV image
        val = new Mat();

        // Zone to search for cones
        searchZone = new Rect(180, 40, 140, 180); // Creates a region in the middle of the frame to look

        areas = new ArrayList<Double>();

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

        parkingList.add(maxIndex + 1);

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

    public void updateJunctionDistance(Mat input) {
        // Use moments to find the center of yellow blobs

        imageROI = input.clone();
        // TODO: Adjust ROI w/ imageROI.adjustROI()
        imageROI = imageROI.adjustROI(
                -200, 0, 0, 0
        );

        Imgproc.cvtColor(imageROI, imageROI, Imgproc.COLOR_RGB2HSV);

        Core.inRange(imageROI, yellowLower, yellowUpper, yellowMask);

        Core.split(imageROI, yellowChannels);

        Core.bitwise_and(yellowChannels.get(2), yellowMask, valYellow);

        Imgproc.findContours(valYellow, yellowContours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        imageROI.release();
        yellowMask.release();
        junctionDistanceInternal = Double.POSITIVE_INFINITY;

        telemetry.update();

        Imgproc.drawContours(input, yellowContours, -1, yellowUpper, 2);

        yellowContourArea = 0;

        for (k = 0; k < yellowContours.size(); k++) {
            yellowContour = yellowContours.get(k);
            if (Imgproc.contourArea(yellowContour) > yellowContourArea) {
                moments = Imgproc.moments(yellowContour, true);
                x = moments.get_m10() / moments.get_m00();
                yellowContourArea = Imgproc.contourArea(yellowContour);
                junctionDistanceInternal = 170 - x;
            }
            yellowContour.release();
        }

        junctionArea = yellowContourArea;

        yellowContours.clear();
        junctionDistance = junctionDistanceInternal;
    }
}