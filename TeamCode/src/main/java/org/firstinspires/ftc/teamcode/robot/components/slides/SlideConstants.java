package org.firstinspires.ftc.teamcode.robot.components.slides;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class SlideConstants {
    /* == Vertical Slide == */
    public static final double VS_TICKS_PER_REV = 384.5;
    public static final double VS_GEAR_RATIO = ((double) 16)/19;
    public static final double VS_PULLEY_CIRCUMFERENCE = 4.409; // in

    public static final double VS_TICKS_PER_IN = VS_TICKS_PER_REV * VS_GEAR_RATIO / VS_PULLEY_CIRCUMFERENCE;

    public static PIDFCoefficients VS_PID = new PIDFCoefficients(0,0,0,0);

    public static double VS_SP_LOW = 10;
    public static double VS_SP_MEDIUM = 15;
    public static double VS_SP_HIGH = 20;

    public static double VS_CLAW_OPEN = 0.7;
    public static double VS_CLAW_CLOSED = 0.5;

    /* == Horizontal Slide == */
    // TODO: Insert correct values for horizontal slide
    public static final double HS_TICKS_PER_REV = 384.5;
    public static final double HS_GEAR_RATIO = 1;
    public static final double HS_PULLEY_CIRCUMFERENCE = 4.409; // in

    public static final double HS_TICKS_PER_IN = HS_TICKS_PER_REV * HS_GEAR_RATIO / HS_PULLEY_CIRCUMFERENCE;

    public static PIDFCoefficients HS_PID = new PIDFCoefficients(0,0,0,0);

    public static double HS_SP_LOW = 10;
    public static double HS_SP_MEDIUM = 15;
    public static double HS_SP_HIGH = 20;

    public static double HS_CLAW_OPEN = 0.5;
    public static double HS_CLAW_CLOSED = 0.8;

    /* == General == */

    public static double SET_POSITION_THRESHOLD = 5;
}
