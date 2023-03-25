package incognito.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

import java.util.Arrays;
import java.util.List;

@Config
public class CameraConstants {
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

    public static Scalar yellowLower = new Scalar(16, 52, 113);
    public static Scalar yellowUpper = new Scalar(41, 255, 255);

    // Junction movement
    public static double JUNCTION_MAX_WIDTH = 35;
    public static double JUNCTION_MIN_WIDTH = 10;
    public static double JUNCTION_Y_POWER_FACTOR = 8;
    public static double JUNCTION_HORIZONTAL_DISTANCE_THRESHOLD = 10;
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