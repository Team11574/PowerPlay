package incognito.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class GenericConstants {
    public static PIDCoefficients JUNCTION_DISTANCE_PID = new PIDCoefficients(-0.003, -0.0001, 0);
    public static PIDCoefficients FORWARD_DISTANCE_PID = new PIDCoefficients(0.008, 0.001, 0.001);//(0.01, 0.002, 0.001);

    public static double JUNCTION_PREFERRED_PIXEL_DISTANCE_CLOSE = 20;
    public static double JUNCTION_PREFERRED_PIXEL_DISTANCE_FAR = 20;
    public static double JUNCTION_MIN_HORIZONTAL_DISTANCE = 1;
    public static double JUNCTION_MAX_HORIZONTAL_DISTANCE = 500;

    public static double JUNCTION_CONE_WIDTH_FACTOR = 5;
    public static double JUNCTION_WIDTH_TO_DISTANCE_FACTOR = 1600;

    public static double JUNCTION_THETA_DISTANCE_FACTOR = 0.05;

    public static double MAX_THETA_POWER = 0.25;
    public static double MAX_Y_POWER = 0.25;

    /* == Front Distance Sensor Junction Distances == */
    public static double FRONT_DS_MAX = 40; // cm
    public static double FRONT_DS_LOW = 31; // with cone, 30 without...
    public static double FRONT_DS_MEDIUM = 31; // with cone, 37 without...
    public static double FRONT_DS_HIGH = 34; // with cone, 34 without...
    public static double FRONT_DS_AVERAGE = (FRONT_DS_LOW + FRONT_DS_MEDIUM + FRONT_DS_HIGH) / 3;
    public static double FRONT_DS_CONE_OFFSET = 3; // cm
    public static double FRONT_DS_THETA_THRESHOLD = 0.2;
    public static double FRONT_DS_DISTANCE_FACTOR = 40;
    public static double FRONT_DS_DISTANCE_THRESHOLD = 1;

    public static double DRIVETRAIN_RAMP_SPEED = 0.1;
    public static double DRIVETRAIN_RIGHT_POWER_MULTIPLIER = 0.96;
}