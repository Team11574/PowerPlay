package org.firstinspires.ftc.teamcode.robot.component.slide;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class SlideConstants {
    /* == Vertical Slide == */
    public static final double VS_TICKS_PER_REV = 384.5;
    public static final double VS_GEAR_RATIO = ((double) 16) / 19;
    public static final double VS_PULLEY_CIRCUMFERENCE = 4.409; // in/rev

    // 73.4383020377 Ticks / In
    public static final double VS_TICKS_PER_IN = VS_TICKS_PER_REV * VS_GEAR_RATIO / VS_PULLEY_CIRCUMFERENCE;
    //    input ticks    | input revolutions  |  output revolutions |
    // ------------------|--------------------|---------------------|  = Input ticks/inch
    // input revolutions | output revolutions |        inch         |

    // TODO: Tune vertical slide
    public static PIDFCoefficients VS_PIDF = new PIDFCoefficients(0, 0, 0, 0);

    public static double VS_KG = 0.04;

    public static double VS_SP_LOW = 20;
    public static double VS_SP_MEDIUM = 35;
    public static double VS_SP_HIGH = 50;
    public static double VS_SP_AUTO = 45;

    public static double VS_CLAW_OPEN = 0.19;
    public static double VS_CLAW_CLOSED = 0;

    public static double VS_FLIP_DOWN = 0.95;
    public static double VS_FLIP_UP = 0.3;

    public static double VS_ENCODER_CENTER = 1000;

    /* == Horizontal Slide == */
    public static final double HS_TICKS_PER_REV = 384.5;
    public static final double HS_GEAR_RATIO = 1;
    public static final double HS_PULLEY_CIRCUMFERENCE = 4.409; // in

    public static final double HS_TICKS_PER_IN = HS_TICKS_PER_REV * HS_GEAR_RATIO / HS_PULLEY_CIRCUMFERENCE;

    public static double HS_MIN_ENCODER = 0;
    public static double HS_MAX_ENCODER = 2100;

    public static double HS_BRAKE_THRESHOLD = 75;

    // TODO: Tune horizontal slide
    public static PIDFCoefficients HS_PIDF = new PIDFCoefficients(0, 0, 0, 0);

    // TODO: Insert correct set positions for horizontal slide
    public static double HS_SP_LOW = 10;
    public static double HS_SP_MEDIUM = 15;
    public static double HS_SP_HIGH = 20;

    public static double HS_CLAW_OPEN = 0.5;
    public static double HS_CLAW_CLOSED = 0.78;


    public static double HS_TURRET_START = 0.88;
    public static double HS_TURRET_MIN = 0.6;
    public static double HS_TURRET_MAX = 1;
    public static double HS_TURRET_SPEED = 0.02;

    public static double HS_HINGE_START = 0.56; // 0.505;
    public static double HS_HINGE_SPEED = 0.01;

    public static double HS_LEVER_IN = 0.7;
    public static double HS_LEVER_MID = 0.5;
    public static double HS_LEVER_FIFTH = 0.315;
    public static double HS_LEVER_FOURTH = 0.28;
    public static double HS_LEVER_THIRD = 0.235;
    public static double HS_LEVER_SECOND = 0.19;
    public static double HS_LEVER_OUT = 0.13;

    public static double HS_LEVER_SPEED = 0.05;
    // Extra Hinge Levelling Variables
    public static final double HS_LEVER_FLAT = 0.267;
    public static final double HS_LEVER_NEG_PI_6 = 0.1;
    public static final double HS_HINGE_FLAT = 0.635;
    public static final double HS_HINGE_NEG_PI_6 = 0.505;
    public static double HS_LEVER_TICKS_PER_RAD = (HS_LEVER_FLAT - HS_LEVER_NEG_PI_6) / (Math.PI / 6);
    public static double HS_HINGE_TICKS_PER_RAD = (HS_HINGE_FLAT - HS_HINGE_NEG_PI_6) / (Math.PI / 6);

    public static double HS_SENSOR_DISTANCE_CM = 2;
    //public static double HS_LEVER_

    /* == Slides General == */
    public static double S_SET_POSITION_THRESHOLD = 20;
    public static double S_RUN_TO_POSITION_POWER = 0.75;
    public static double S_AUTO_EXTEND_POWER = 0.75;


    // REMOVE ONCE CURRENT ALERT TESTING IS FINISHED
    public static double CURRENT_ALERT = 10000;
    public static double CURRENT_ALERT_STOPPED = 7500;
    public static double VELOCITY_STOP_THRESHOLD = 100;
}