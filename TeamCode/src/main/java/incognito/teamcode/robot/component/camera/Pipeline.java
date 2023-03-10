package incognito.teamcode.robot.component.camera;

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
import java.util.ConcurrentModificationException;
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
    int j;
    int maxIndex;

    double area;
    double area2;
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
        colorRanges.add(CameraConstants.orangeThreshold);
        colorRanges.add(CameraConstants.purpleThreshold);
        colorRanges.add(CameraConstants.greenThreshold);

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
        try {
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
                    -searchZone.y,
                    -(searchZone.height + searchZone.y) + 320,
                    -searchZone.x,
                    -(searchZone.width + searchZone.x) + 240
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

            //telemetry.addData("Array", areas);
            //telemetry.addData("Orange Area", areas.get(0));
            //telemetry.addData("Purple Area", areas.get(1));
            //telemetry.addData("Green Area", areas.get(2));
            telemetry.addData("Max Area", maxArea);
            telemetry.addData("Zone", maxIndex + 1);
            telemetry.update();

            imageROI.release();
            return input;
        } catch (ConcurrentModificationException e) {
            e.printStackTrace();
            return input;
        }
    }

    public double getArea(Mat input, List<Scalar> colorRange) {
        try {
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
        } catch (ConcurrentModificationException c) {
            return 0;
        }
    }

    /**
     * Gets the parking spot from the parking list
     * (using last frames, specified by SAMPLES)
     *
     * @return parking spot
     */
    public int getParkingSpot() {
        try {
            if (parkingList.size() > SAMPLES) {
                return mode(parkingList.subList(Math.max(parkingList.size() - (SAMPLES + 1), 0), parkingList.size()));
            } else {
                return mode(parkingList);
            }
        } catch (ConcurrentModificationException c) {
            return 2;
        }
    }

    /**
     * Gets the mode of a list
     *
     * @param a - list to get mode of
     * @return mode
     */
    static int mode(List<Integer> a) {
        try {
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
        } catch (ConcurrentModificationException c) {
            return 2;
        }
    }

    /**
     * Stops the pipeline
     */
    public void stop() {
        stopped = true;
    }

    /*public double get_junction_distance(Mat input) {
        // Detect edges of a yellow rectangle

        // Convert to HSV
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

        // Threshold for yellow
        Core.inRange(input, new Scalar(20, 100, 100), new Scalar(30, 255, 255), input);

        // Find edges of yellow rectangle
        Imgproc.Canny(input, input, 100, 200);

        // Find center of yellow rectangle
        Moments m = Imgproc.moments(input, true);
        Point center = new Point(m.get_m10() / m.get_m00(), m.get_m01() / m.get_m00());

        // Find distance from center of image to center of yellow rectangle
        double distance = Math.sqrt(Math.pow(center.x - 120, 2) + Math.pow(center.y - 160, 2));

        return 0;
    }*/
}