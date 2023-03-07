package org.firstinspires.ftc.teamcode.util;

public class Generic {
    public static boolean withinThreshold(double currentValue, double targetValue, double threshold) {
        return Math.abs(currentValue - targetValue) <= threshold;
    }

    public static <T extends Comparable<T>> T clamp(T value, T min, T max) {
        if (value.compareTo(min) < 0) return min;
        else if (value.compareTo(max) > 0) return max;
        else return value;
    }
}
