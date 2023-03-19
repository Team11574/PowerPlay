package incognito.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

import java.util.Arrays;
import java.util.List;

@Config
public class CameraConstants {
    // === Color Thresholds ===

    public static Scalar greenLow = new Scalar(40, 40, 0);
    public static Scalar greenHigh = new Scalar(70, 225, 225);
    public static final List<Scalar> greenThreshold = Arrays.asList(greenLow, greenHigh);

    public static Scalar orangeLow = new Scalar(20, 40, 0); // Orange min HSV
    public static Scalar orangeHigh = new Scalar(30, 255, 255); // Orange max HSV
    public static final List<Scalar> orangeThreshold = Arrays.asList(orangeLow, orangeHigh);

    public static Scalar purpleLow = new Scalar(130, 40, 0);
    public static Scalar purpleHigh = new Scalar(150, 255, 255);
    public static final List<Scalar> purpleThreshold = Arrays.asList(purpleLow, purpleHigh);

    public static Scalar yellowLower = new Scalar(16, 123, 140);
    public static Scalar yellowUpper = new Scalar(41, 255, 255);

    public static double JUNCTION_MAX_WIDTH = 30;
    public static double JUNCTION_MIN_WIDTH = 5;
    public static double JUNCTION_Y_POWER_FACTOR = 8;
    public static double JUNCTION_DISTANCE_THRESHOLD = 10;
    public static double JUNCTION_THETA_POWER_FACTOR = -0.005;
}