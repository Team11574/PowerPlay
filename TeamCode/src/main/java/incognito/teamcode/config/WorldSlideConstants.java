package incognito.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class WorldSlideConstants {
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

    public static double VS_LEVER_WAIT_TIME = 500; // ms
    public static double VS_HINGE_WAIT_TIME = 150; // ms
    public static double VS_CLAW_WAIT_TIME = 100; // ms

    // TODO: Set correct slide positions (UPDATE after new slide is added)
    public static int VS_SLIDE_INTAKE = 0;
    public static int VS_SLIDE_LOW = 450;
    public static int VS_SLIDE_MEDIUM = 50;
    public static int VS_SLIDE_HIGH = 875;

    public static double VS_OFFSET_FACTOR = 100;


    // TODO: Set correct positions for vertical lever
    public static double VS_LEVER_START = 0.03; // Start = lever in
    public static double VS_LEVER_END = 1; // End = lever out
    public static double VS_LEVER_INTAKE = VS_LEVER_START;
    public static double VS_LEVER_LOW = .93;
    public static double VS_LEVER_MEDIUM = 0.6; //0.6;
    public static double VS_LEVER_HIGH = 0.55;


    // TODO: Set correct positions for vertical hinge
    public static double VS_HINGE_START = 0.08;//0.1; // Start = min
    public static double VS_HINGE_END = 0.86;//0.8; // End = max
    public static double VS_HINGE_INTAKE = 0.22;
    public static double VS_HINGE_LOW_UP = 0.77;
    public static double VS_HINGE_LOW_DOWN = 0.7;
    public static double VS_HINGE_MEDIUM_UP = 0.42;
    public static double VS_HINGE_MEDIUM_DOWN = 0.25; //0.31;
    public static double VS_HINGE_HIGH_UP = 0.35;
    public static double VS_HINGE_HIGH_DOWN = 0.24;

    public static double VS_HINGE_PAUSE_TIME = 250; // ms
    public static double VS_HINGE_TO_INTAKE_TIME = 300; // ms
    public static double VS_HINGE_TO_INTAKE_TIME_LOW = 750; // ms

    public enum VS_TIME_TO {
        INTAKE_TO_LOW(VS_INTAKE_TO_LOW_TIME),
        INTAKE_TO_MEDIUM(VS_INTAKE_TO_MEDIUM_TIME),
        INTAKE_TO_HIGH(VS_INTAKE_TO_HIGH_TIME),
        HIGH_TO_INTAKE(VS_HIGH_TO_INTAKE_TIME),
        MEDIUM_TO_INTAKE(VS_MEDIUM_TO_INTAKE_TIME),
        DEFAULT(VS_DEFAULT_TIME)
        ;
        private final double time;

        VS_TIME_TO(double time) {
            this.time = time;
        }

        public static double getTime(String pos1, String pos2) {
            try {
                return VS_TIME_TO.valueOf(pos1 + "_TO_" + pos2).time;
            } catch (IllegalArgumentException e1) {
                try {
                    return VS_TIME_TO.valueOf(pos2 + "_TO_" + pos1).time;
                } catch (IllegalArgumentException e2) {
                    return DEFAULT.time;
                }
            }
        }
    }

    public static double VS_INTAKE_TO_LOW_TIME = 750; // ms
    public static double VS_INTAKE_TO_MEDIUM_TIME = 400; // ms
    public static double VS_MEDIUM_TO_INTAKE_TIME = 450; // ms
    public static double VS_INTAKE_TO_HIGH_TIME = 350; // ms
    public static double VS_HIGH_TO_INTAKE_TIME = 400; // ms
    public static double VS_DEFAULT_TIME = 450; // ms

    // TODO: Find correct vertical claw open/closed positions
    public static double VS_CLAW_WIDE = 0.2;//0.2;
    public static double VS_CLAW_OPEN = 0.11;//0.29;
    public static double VS_CLAW_CLOSED = 0; //0.03;//0.35;

    public static double VS_CLAW_HANDOFF_SPEED = 100;
    public static double VS_CLAW_DROP_SPEED = 200;
    public static double VS_CLAW_GRAB_SPEED = 0;

    public static double HS_WAIT_OUT_SPEED = 100;
    public static double HS_CLAW_DROP_SPEED = 150;
    public static double HS_CLAW_GRAB_SPEED = 25;




    /* == Horizontal Slide == */
    public static final double HS_TICKS_PER_REV = 384.5;
    public static final double HS_GEAR_RATIO = 1;
    public static final double HS_PULLEY_CIRCUMFERENCE = 4.409; // in

    public static final double HS_TICKS_PER_IN = HS_TICKS_PER_REV * HS_GEAR_RATIO / HS_PULLEY_CIRCUMFERENCE;

    public static double HS_SLIDE_MIN = 0;
    public static double HS_SLIDE_MAX = 2000;

    public static double HS_LEVER_WAIT_TIME = 100; //500; // ms
    public static double HS_HINGE_WAIT_TIME = 100; //150; // ms
    public static double HS_CLAW_WAIT_TIME = 150; //250; // ms

    public static double HS_BRAKE_THRESHOLD = 75;

    // TODO: Tune horizontal slide
    public static PIDFCoefficients HS_PIDF = new PIDFCoefficients(0, 0, 0, 0);

    // TODO: Insert correct set positions for horizontal slide

    // TODO: Find correct values
    public static double HS_CLAW_OPEN = 0.4;
    public static double HS_CLAW_CLOSED = 0.15;

    public static double HS_HINGE_IN = 0.73; // .86;
    public static double HS_HINGE_GROUND = 0.755;
    public static double HS_HINGE_OUT = 0.63; // 0.715;
    public static double HS_HINGE_SECOND = 0.54; // 0.605;
    public static double HS_HINGE_THIRD = 0.5; // 0.53;
    public static double HS_HINGE_FOURTH = 0.45;
    public static double HS_HINGE_FIFTH = 0.36; // 0.28;
    public static double HS_HINGE_MID = 0.47; //0.455;
    public static double HS_HINGE_UP_LOW = 0.2;
    public static double HS_HINGE_MAX = 0.765;
    public static double HS_HINGE_MIN = 0.125;

    public static double HS_LEVER_START = 0.88;
    public static double HS_LEVER_MID = 0.8;
    public static double HS_LEVER_FIFTH = 0.48;
    public static double HS_LEVER_FOURTH = 0.35;
    public static double HS_LEVER_THIRD = 0.26;
    public static double HS_LEVER_SECOND = 0.18;
    public static double HS_LEVER_END = 0.04;

    /* == Horizontal Slide Cone Distance == */
    public static double HS_DS_CONE_DISTANCE_CM = 6; // cm
    public static double HS_DS_CONE_SUPER_DISTANCE_CM = 7; // cm
    public static int HS_CONE_JUMP_DISTANCE = 5; // ticks

    public static int HS_SLIDE_AUTO_OUT_POSITION = 750;


    /* == Slides General == */
    public static double S_SET_POSITION_THRESHOLD = 20;
    public static double S_RUN_TO_POSITION_POWER = 1;
    // TODO: UPDATE
    public static double S_ENCODER_CENTER_ISH = 300;
    public static double S_JOYSTICK_THRESHOLD = 0.1;


    // REMOVE ONCE CURRENT ALERT TESTING IS FINISHED
    public static double CURRENT_ALERT = 10000;
    public static double CURRENT_ALERT_STOPPED = 7500;
    public static double VELOCITY_STOP_THRESHOLD = 100;
}