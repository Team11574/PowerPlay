package incognito.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class GenericConstants {
    public static PIDCoefficients JUNCTION_DISTANCE_PID = new PIDCoefficients(-0.005, 0, 0.001);

    /* == Front Distance Sensor Junction Distances == */
    public static double FRONT_DS_MAX = 80; // cm
    public static double FRONT_DS_LOW = 26; // with cone, 30 without...
    public static double FRONT_DS_MEDIUM = 32; // with cone, 37 without...
    public static double FRONT_DS_HIGH = 30; // with cone, 34 without...
    public static double FRONT_DS_AVERAGE = (FRONT_DS_LOW + FRONT_DS_MEDIUM + FRONT_DS_HIGH) / 3;
    public static double FRONT_DS_CONE_OFFSET = 5; // cm
    public static double FRONT_DS_THETA_THRESHOLD = 0.05;
    public static double FRONT_DS_DISTANCE_FACTOR = 20;
    public static double FRONT_DS_DISTANCE_THRESHOLD = 1;

    public static double DRIVETRAIN_RAMP_SPEED = 0.1;
}