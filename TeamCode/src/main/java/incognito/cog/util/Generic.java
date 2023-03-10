package incognito.cog.util;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Generic {
    public static boolean withinThreshold(double currentValue, double targetValue, double threshold) {
        return Math.abs(currentValue - targetValue) <= threshold;
    }

    public static <T extends Comparable<T>> T clamp(T value, T min, T max) {
        if (value.compareTo(min) < 0) return min;
        else if (value.compareTo(max) > 0) return max;
        else return value;
    }

    /**
     * Find the Vector2d that is the midpoint between two Vector2d.
     *
     * @param first  Vector2d first position.
     * @param second Vector2d second position.
     * @return Vector2d midpoint.
     */
    public static Vector2d midpoint(Vector2d first, Vector2d second) {
        return new Vector2d(
                (first.getX() + second.getX()) / 2,
                (first.getY() + second.getY()) / 2
        );
    }
}