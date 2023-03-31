package incognito.teamcode.robot;

import static incognito.teamcode.config.WorldSlideConstants.HS_CLAW_CLOSED;
import static incognito.teamcode.config.WorldSlideConstants.HS_CLAW_OPEN;
import static incognito.teamcode.config.WorldSlideConstants.HS_HINGE_END;
import static incognito.teamcode.config.WorldSlideConstants.HS_HINGE_MID;
import static incognito.teamcode.config.WorldSlideConstants.HS_HINGE_START;
import static incognito.teamcode.config.WorldSlideConstants.HS_LEVER_END;
import static incognito.teamcode.config.WorldSlideConstants.HS_LEVER_FIFTH;
import static incognito.teamcode.config.WorldSlideConstants.HS_LEVER_FOURTH;
import static incognito.teamcode.config.WorldSlideConstants.HS_LEVER_MID;
import static incognito.teamcode.config.WorldSlideConstants.HS_LEVER_SECOND;
import static incognito.teamcode.config.WorldSlideConstants.HS_LEVER_START;
import static incognito.teamcode.config.WorldSlideConstants.HS_LEVER_THIRD;
import static incognito.teamcode.config.WorldSlideConstants.HS_MAX_ENCODER;
import static incognito.teamcode.config.WorldSlideConstants.HS_MIN_ENCODER;
import static incognito.teamcode.config.WorldSlideConstants.VS_CLAW_CLOSED;
import static incognito.teamcode.config.WorldSlideConstants.VS_CLAW_OPEN;
import static incognito.teamcode.config.WorldSlideConstants.VS_HINGE_END;
import static incognito.teamcode.config.WorldSlideConstants.VS_HINGE_HIGH_DOWN;
import static incognito.teamcode.config.WorldSlideConstants.VS_HINGE_HIGH_UP;
import static incognito.teamcode.config.WorldSlideConstants.VS_HINGE_INTAKE;
import static incognito.teamcode.config.WorldSlideConstants.VS_HINGE_LOW_DOWN;
import static incognito.teamcode.config.WorldSlideConstants.VS_HINGE_LOW_UP;
import static incognito.teamcode.config.WorldSlideConstants.VS_HINGE_MEDIUM_DOWN;
import static incognito.teamcode.config.WorldSlideConstants.VS_HINGE_MEDIUM_UP;
import static incognito.teamcode.config.WorldSlideConstants.VS_HINGE_START;
import static incognito.teamcode.config.WorldSlideConstants.VS_LEVER_END;
import static incognito.teamcode.config.WorldSlideConstants.VS_LEVER_HIGH;
import static incognito.teamcode.config.WorldSlideConstants.VS_LEVER_INTAKE;
import static incognito.teamcode.config.WorldSlideConstants.VS_LEVER_LOW;
import static incognito.teamcode.config.WorldSlideConstants.VS_LEVER_MEDIUM;
import static incognito.teamcode.config.WorldSlideConstants.VS_LEVER_START;
import static incognito.teamcode.config.WorldSlideConstants.VS_SLIDE_HIGH;
import static incognito.teamcode.config.WorldSlideConstants.VS_SLIDE_INTAKE;
import static incognito.teamcode.config.WorldSlideConstants.VS_SLIDE_LOW;
import static incognito.teamcode.config.WorldSlideConstants.VS_SLIDE_MEDIUM;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import incognito.cog.actions.Scheduler;
import incognito.cog.hardware.component.drive.Drivetrain;
import incognito.cog.robot.RobotCog;
import incognito.teamcode.robot.component.arm.Arm;
import incognito.teamcode.robot.component.arm.HorizontalArm;
import incognito.teamcode.robot.component.arm.VerticalArm;
import incognito.teamcode.robot.component.camera.AutoCamera;
import incognito.teamcode.robot.component.servoImplementations.Claw;
import incognito.teamcode.robot.component.servoImplementations.HorizontalHinge;
import incognito.teamcode.robot.component.servoImplementations.VerticalHinge;
import incognito.teamcode.robot.component.servoImplementations.Lever;
import incognito.teamcode.robot.component.slide.HorizontalSlide;
import incognito.teamcode.robot.component.slide.VerticalSlide;


public class WorldRobot extends RobotCog {
    // ===== Instance Variables =====

    boolean cameraEnabled;

    // Inherit hardwareMap and telemetry from OpMode
    HardwareMap hardwareMap;
    Telemetry telemetry;

    // -- Components --
    public AutoCamera autoCamera;
    public Drivetrain drivetrain;

    public VerticalArm verticalArm;
    private VerticalSlide verticalSlide;
    private Claw verticalClaw;
    private Lever verticalLever;
    private VerticalHinge verticalHinge;

    //public ModernRoboticsI2cRangeSensor frontDistanceSensor;
    public DistanceSensor frontDistanceSensor;

    public HorizontalArm horizontalArm;
    private HorizontalSlide horizontalSlide;
    private Claw horizontalClaw;
    private Lever horizontalLever;
    private HorizontalHinge horizontalHinge;
    public DistanceSensor horizontalDistanceSensor;

    public Scheduler horizontalScheduler;
    public Scheduler verticalScheduler;


    public WorldRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, false);
    }

    public WorldRobot(HardwareMap hardwareMap, Telemetry telemetry, boolean cameraEnabled) {
        super(hardwareMap, telemetry);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.cameraEnabled = cameraEnabled;

        if (cameraEnabled)
            autoCamera = new AutoCamera(hardwareMap, telemetry);

        horizontalScheduler = new Scheduler();
        verticalScheduler = new Scheduler();
        //frontDistanceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front_DS");
        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "front_DS");
        configureDrivetrain();
        configureHorizontalSlide();
        configureHorizontalArm();
        configureVerticalSlide();
        configureVerticalArm();
    }

    private void configureDrivetrain() {
        DcMotorEx DT_frontRight_M = hardwareMap.get(DcMotorEx.class, "DT_frontRight_M");
        DcMotorEx DT_backRight_M = hardwareMap.get(DcMotorEx.class, "DT_backRight_M");
        DcMotorEx DT_frontLeft_M = hardwareMap.get(DcMotorEx.class, "DT_frontLeft_M");
        DcMotorEx DT_backLeft_M = hardwareMap.get(DcMotorEx.class, "DT_backLeft_M");
        drivetrain = new Drivetrain(hardwareMap, telemetry, DT_frontRight_M, DT_backRight_M, DT_frontLeft_M, DT_backLeft_M);
    }

    private void configureVerticalSlide() {
        DcMotorEx VS_slideRight_M = hardwareMap.get(DcMotorEx.class, "VS_slideRight_M");
        DcMotorEx VS_slideLeft_M = hardwareMap.get(DcMotorEx.class, "VS_slideLeft_M");
        VS_slideLeft_M.setDirection(DcMotorEx.Direction.REVERSE);
        DigitalChannel VS_limitSwitch_D = hardwareMap.get(DigitalChannel.class, "VS_limitSwitch_D");
        verticalSlide = new VerticalSlide(hardwareMap, telemetry, new DcMotorEx[]{VS_slideLeft_M, VS_slideRight_M}, VS_limitSwitch_D);
        verticalSlide.addSetPositions(new int[]{VS_SLIDE_INTAKE, VS_SLIDE_LOW, VS_SLIDE_MEDIUM, VS_SLIDE_HIGH});
        verticalSlide.goToSetPosition(VerticalSlide.Position.INTAKE);
    }

    private void configureVerticalArm() {
        Servo VS_hinge_S = hardwareMap.get(Servo.class, "VS_hinge_S");
        verticalHinge = new VerticalHinge(hardwareMap, telemetry, VS_hinge_S,
                new double[] {
                        VS_HINGE_INTAKE,
                        VS_HINGE_LOW_UP, VS_HINGE_LOW_DOWN,
                        VS_HINGE_MEDIUM_UP, VS_HINGE_MEDIUM_DOWN,
                        VS_HINGE_HIGH_UP, VS_HINGE_HIGH_DOWN
                }, VS_HINGE_START, VS_HINGE_END);

        Servo VS_lever_S = hardwareMap.get(Servo.class, "VS_lever_S");
        verticalLever = new Lever(hardwareMap, telemetry, VS_lever_S, new double[] {
                VS_LEVER_INTAKE, VS_LEVER_LOW, VS_LEVER_MEDIUM, VS_LEVER_HIGH
        }, VS_LEVER_START, VS_LEVER_END
        );

        Servo VS_claw_S = hardwareMap.get(Servo.class, "VS_claw_S");
        verticalClaw = new Claw(hardwareMap, telemetry, VS_claw_S, VS_CLAW_OPEN, VS_CLAW_CLOSED);

        verticalArm = new VerticalArm(verticalSlide, verticalLever, verticalHinge, verticalClaw);
    }

    private void configureHorizontalArm() {
        Servo HS_hinge_S = hardwareMap.get(Servo.class, "HS_hinge_S");
        horizontalHinge = new HorizontalHinge(hardwareMap, telemetry, HS_hinge_S,
                new double[] {
                        HS_HINGE_START,
                        HS_HINGE_MID,
                        HS_HINGE_END
                }, HS_HINGE_END, HS_HINGE_START);
        //horizontalHinge.setOffsetFactor(HS_HINGE_SPEED);

        Servo HS_lever_S = hardwareMap.get(Servo.class, "HS_lever_S");
        horizontalLever = new Lever(hardwareMap, telemetry, HS_lever_S,
                new double[]{HS_LEVER_START,
                        HS_LEVER_MID,
                        HS_LEVER_FIFTH,
                        HS_LEVER_FOURTH,
                        HS_LEVER_THIRD,
                        HS_LEVER_SECOND,
                        HS_LEVER_END},
                HS_LEVER_END, HS_LEVER_START);
        //horizontalLever.setOffsetFactor(HS_LEVER_SPEED);

        Servo HS_claw_S = hardwareMap.get(Servo.class, "HS_claw_S");
        horizontalClaw = new Claw(hardwareMap, telemetry, HS_claw_S, HS_CLAW_OPEN, HS_CLAW_CLOSED);

        horizontalDistanceSensor = hardwareMap.get(DistanceSensor.class, "HS_DS");

        horizontalArm = new HorizontalArm(horizontalSlide, horizontalLever, horizontalHinge, horizontalClaw, horizontalDistanceSensor);
    }

    private void configureHorizontalSlide() {
        DcMotorEx HS_slide_M = hardwareMap.get(DcMotorEx.class, "HS_slide_M");
        horizontalSlide = new HorizontalSlide(hardwareMap, telemetry, HS_slide_M, HS_MIN_ENCODER, HS_MAX_ENCODER);
        horizontalSlide.goToSetPosition(HorizontalSlide.Position.IN);
    }

    // Turned to public variables
    // public Drivetrain getDrivetrain() { return drivetrain; }
    // public HorizontalSlide getHorizontalSlide() { return horizontalSlide; };
    // public VerticalSlide getVerticalSlide() { return verticalSlide; };

    public int getParkingSpot() {
        if (cameraEnabled)
            return autoCamera.getParkingSpot();
        // Default spot is 2
        return 2;
    }/*

    public void moveLever(double amount) {
        moveLever(amount, true);
    }

    public void moveLever(double amount, boolean updateLastPosition) {
        lever.offsetPosition(amount, updateLastPosition);
    }

    public void moveTurret(double amount) {
        turret.offsetPosition(amount);
    }

    public void retractArm() {
        retractArm(true, true);
    }

    public void retractArm(boolean doReturn) {
        retractArm(doReturn, false);
    }

    public void retractArm(boolean doReturn, boolean drop) {
        if (!isRetracting) {
            isRetracting = true;
            // Lever mid
            horizontalClaw.close();
            lever.goToSetPosition(Lever.LeverPosition.MID, false);
            horizontalScheduler.linearSchedule(
                    when -> true,
                    then -> {
                        horizontalSlide.goToSetPosition(0);
                        hinge.goToSetPosition(0, false);
                        turret.goToSetPosition(0);
                    },
                    250
            );
            horizontalScheduler.linearSchedule(
                    //when -> horizontalSlide.goToPositionConstant(0),
                    when -> horizontalSlide.atSetPosition(),
                    // Lever in
                    then -> {
                        lever.goToSetPosition(Lever.LeverPosition.IN, false);
                        if (!drop && !doReturn) {
                            isRetracting = false;
                        }
                    }
            );
            if (drop) {
                horizontalScheduler.linearSchedule(
                        when -> true,
                        then -> {
                            horizontalClaw.open();
                            if (!doReturn) {
                                isRetracting = false;
                            }
                        },
                        500
                );
            }
            if (doReturn) {
                horizontalScheduler.linearSchedule(
                        when -> true,
                        then -> returnOut(),
                        500
                );
                horizontalScheduler.linearSchedule(
                        when -> horizontalSlide.atSetPosition(),
                        then -> isRetracting = false
                );
            }

        } // else, do nothing. We don't want to double schedule movements.
    }

    public void returnOut() {
        horizontalSlide.goToLastPosition();
        // Lever out
        lever.goToLastPosition();
        // Hinge back
        hinge.goToLastPosition();
    }

    public void waitForRetract() {
        // NOT ASYNC
        while (isRetracting) {
            //horizontalScheduler.update();
            update();
        }
    }

    public void waitForDeposit() {
        // NOT ASYNC
        while (isDepositing) {
            //verticalScheduler.update();
            update();
        }
    }

    public void depositCone() {
        if (!isDepositing) {
            isDepositing = true;
            verticalFlip.flipDown();
            verticalScheduler.linearSchedule(
                    when -> true,
                    then -> verticalClaw.open(),
                    750
            );
            verticalScheduler.linearSchedule(
                    when -> true,
                    then -> {
                        verticalFlip.flipUp();
                        verticalClaw.close();
                        isDepositing = false;
                    },
                    250
            );
            verticalScheduler.linearSchedule(
                    when -> true,
                    then -> {
                        verticalClaw.open();
                    },
                    750
            );
        }
    }

    public void levelHinge() {
        *//* OLD
        // The angle of the lever in radians
        double leverAngle = (lever.getPosition() - HS_LEVER_FLAT) / HS_LEVER_TICKS_PER_RAD;

        // Subtract from 2PI to keep hinge parallel to ground
        double hingeParallel = leverAngle;

        // The position of the hinge in ticks
        double newHingePosition = hingeParallel * HS_HINGE_TICKS_PER_RAD + HS_HINGE_FLAT;

        hinge.setPosition(newHingePosition);
         *//*
        double hingePosition = lever.getPosition() - HS_LEVER_FLAT + HS_HINGE_FLAT + 0.05;
        if (hingePosition < 1) {
            hinge.setPosition(hingePosition);
        } else {
            hinge.goToSetPosition(0);
        }
    }

    public void extend() {
        isExtending = true;
        dropHorizontalArm();
        horizontalScheduler.linearSchedule(
                when -> true,
                then -> {
                    for (DcMotorEx motor : horizontalSlide.motors) {
                        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    horizontalSlide.setPower(S_AUTO_EXTEND_POWER);
                },
                500
        );

        // Once at distance, stop moving the slide but
        // say we are still in the process of extending
        // for the sake of the X toggle
        horizontalScheduler.linearSchedule(
                when -> nearCone(),
                then -> horizontalSlide.setPower(0)
        );
    }

    public void finishExtend() {
        horizontalClaw.close();
        stopExtend();
        retractArm(false, false);
    }

    public void stopExtend() {
        horizontalSlide.setPower(0);
        isExtending = false;
    }

    public void dropHorizontalArm() {
        // TODO: Figure out how to input what height the lever is at
        horizontalClaw.open();
        lever.goToSetPosition(Lever.LeverPosition.OUT, true);
        levelHinge();
    }

    public boolean nearCone() {
        return nearCone(HS_SENSOR_DISTANCE_CM);
    }

    public boolean nearCone(double distance) {
        return horizontalDistanceSensor.getDistance(DistanceUnit.CM) < distance;
    }*/

    public void update() {
        verticalArm.update();
        horizontalArm.update();
        verticalSlide.update();
        horizontalSlide.update();
        horizontalScheduler.update();
        verticalScheduler.update();
    }
}