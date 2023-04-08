package incognito.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

import java.util.Arrays;
import java.util.List;

@Config
public class CameraConstants {

    public static int VIEWPORT_WIDTH = 432;
    public static int VIEWPORT_HEIGHT = 240;

    // === Color Thresholds ===
    public static Scalar greenLow = new Scalar(43, 14, 99);
    public static Scalar greenHigh = new Scalar(94, 197, 204);
    public static final List<Scalar> greenThreshold = Arrays.asList(greenLow, greenHigh);

    public static Scalar orangeLow = new Scalar(0, 50, 128); // Orange min HSV
    public static Scalar orangeHigh = new Scalar(15, 255, 255); // Orange max HSV
    public static final List<Scalar> orangeThreshold = Arrays.asList(orangeLow, orangeHigh);

    public static Scalar purpleLow = new Scalar(115, 62, 118);
    public static Scalar purpleHigh = new Scalar(140, 174, 255);
    public static final List<Scalar> purpleThreshold = Arrays.asList(purpleLow, purpleHigh);

    public static Scalar yellowLower = new Scalar(16, 85, 113);
    public static Scalar yellowUpper = new Scalar(31, 255, 255);
    public static final List<Scalar> yellowThreshold = Arrays.asList(yellowLower, yellowUpper);

    public static Scalar redLower = new Scalar(140, 31, 62);
    public static Scalar redUpper = new Scalar(183, 255, 255);
    public static final List<Scalar> redThreshold = Arrays.asList(redLower, redUpper);

    public static Scalar blueLower = new Scalar(105, 101, 25);
    public static Scalar blueUpper = new Scalar(123, 255, 255);
    public static final List<Scalar> blueThreshold = Arrays.asList(blueLower, blueUpper);

    public static double YELLOW_WIDTH_THRESHOLD = 20;
    public static double RED_WIDTH_THRESHOLD = 50;
    public static double BLUE_WIDTH_THRESHOLD = 50;

    // Junction movement

    public static double JUNCTION_MAX_WIDTH = 35;
    public static double JUNCTION_MIN_WIDTH = 10;
    public static double JUNCTION_Y_POWER_FACTOR = 8;
    public static double JUNCTION_MIN_HORIZONTAL_DISTANCE = 10;
    public static double JUNCTION_MAX_HORIZONTAL_DISTANCE = 500;
    public static double JUNCTION_THETA_POWER_FACTOR = -0.005;

    public static double tagSize = 0.04;
    // For 432/240 (ish)
    public static double fx = 1399.628695878517/3;
    public static double fy = 1399.2193585515058/3;
    public static double cx = 653.018726955712/3;
    public static double cy = 364.4540357506378/3;

    public static double aprilWeight = 1;
    public static double colorWeight = 0.9;
}