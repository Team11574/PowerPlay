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

    int i;
    int maxIndex;

    double area;
    double maxArea;

    ArrayList<Integer> parkingList = new ArrayList<Integer>();

    // Number of frames to include in parking decision
    final int SAMPLES = 10;


    public Pipeline(Telemetry telem) {
        telemetry = telem;

        contours = new ArrayList<MatOfPoint>();
        mask = new Mat();
        maxContour = new MatOfPoint();

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

        if (imageROI != null)
            imageROI.release();

        // Clone input to imageROI
        imageROI = input.clone();

        // Makes input to HSV from RGB image
        Imgproc.cvtColor(imageROI, imageROI, Imgproc.COLOR_RGB2HSV);


        // Creates a region of interest in the middle of the frame
        imageROI = imageROI.adjustROI(
                searchZone.y,
                searchZone.y + searchZone.height,
                searchZone.x,
                searchZone.x + searchZone.width
        );

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

        for (i = 0; i < contours.size(); i++) {
            contour = contours.get(i);
            area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
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
        }
        else {
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
