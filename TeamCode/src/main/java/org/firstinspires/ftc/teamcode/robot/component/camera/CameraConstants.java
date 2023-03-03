package org.firstinspires.ftc.teamcode.robot.component.camera;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

import java.util.Arrays;
import java.util.List;

@Config
public class CameraConstants {
    // === Color Thresholds ===
    public static Scalar orangeLow = new Scalar(20, 40, 0); // Orange min HSV
    public static Scalar orangeHigh = new Scalar(30, 255, 255); // Orange max HSV
    public static final List<Scalar> orangeThreshold = Arrays.asList(orangeLow, orangeHigh);

    public static Scalar purpleLow = new Scalar(130, 40, 0);
    public static Scalar purpleHigh = new Scalar(150, 255, 255);
    public static final List<Scalar> purpleThreshold = Arrays.asList(purpleLow, purpleHigh);

    public static Scalar greenLow = new Scalar(40, 40, 0);
    public static Scalar greenHigh = new Scalar(70, 225, 225);
    public static final List<Scalar> greenThreshold = Arrays.asList(greenLow, greenHigh);
}
