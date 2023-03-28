package incognito.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class GenericConstants {
    public static PIDCoefficients JUNCTION_DISTANCE_PID = new PIDCoefficients(-0.005, 0, 0);
}