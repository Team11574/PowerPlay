package incognito.teamcode.robot;

import static incognito.teamcode.config.SlideConstants.HS_CLAW_CLOSED;
import static incognito.teamcode.config.SlideConstants.HS_CLAW_OPEN;
import static incognito.teamcode.config.SlideConstants.HS_HINGE_FLAT;
import static incognito.teamcode.config.SlideConstants.HS_HINGE_SPEED;
import static incognito.teamcode.config.SlideConstants.HS_HINGE_IN;
import static incognito.teamcode.config.SlideConstants.HS_LEVER_FIFTH;
import static incognito.teamcode.config.SlideConstants.HS_LEVER_FLAT;
import static incognito.teamcode.config.SlideConstants.HS_LEVER_FOURTH;
import static incognito.teamcode.config.SlideConstants.HS_LEVER_IN;
import static incognito.teamcode.config.SlideConstants.HS_LEVER_MID;
import static incognito.teamcode.config.SlideConstants.HS_LEVER_OUT;
import static incognito.teamcode.config.SlideConstants.HS_LEVER_SECOND;
import static incognito.teamcode.config.SlideConstants.HS_LEVER_SPEED;
import static incognito.teamcode.config.SlideConstants.HS_LEVER_THIRD;
import static incognito.teamcode.config.SlideConstants.HS_MAX_ENCODER;
import static incognito.teamcode.config.SlideConstants.HS_MIN_ENCODER;
import static incognito.teamcode.config.SlideConstants.HS_SENSOR_DISTANCE_CM;
import static incognito.teamcode.config.SlideConstants.HS_TURRET_MAX;
import static incognito.teamcode.config.SlideConstants.HS_TURRET_MIN;
import static incognito.teamcode.config.SlideConstants.HS_TURRET_SPEED;
import static incognito.teamcode.config.SlideConstants.HS_TURRET_START;
import static incognito.teamcode.config.SlideConstants.S_AUTO_EXTEND_POWER;
import static incognito.teamcode.config.SlideConstants.VS_CLAW_CLOSED;
import static incognito.teamcode.config.SlideConstants.VS_CLAW_OPEN;
import static incognito.teamcode.config.SlideConstants.VS_FLIP_DOWN;
import static incognito.teamcode.config.SlideConstants.VS_FLIP_UP;
import static incognito.teamcode.config.SlideConstants.VS_SP_AUTO;
import static incognito.teamcode.config.SlideConstants.VS_SP_HIGH;
import static incognito.teamcode.config.SlideConstants.VS_SP_LOW;
import static incognito.teamcode.config.SlideConstants.VS_SP_MEDIUM;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import incognito.cog.actions.Scheduler;
import incognito.cog.hardware.component.drive.Drivetrain;
import incognito.cog.hardware.component.servo.ContinuousServo;
import incognito.cog.robot.RobotCog;
import incognito.teamcode.robot.component.camera.AutoCamera;
import incognito.teamcode.robot.component.servoImplementations.Claw;
import incognito.teamcode.robot.component.servoImplementations.Flipper;
import incognito.teamcode.robot.component.servoImplementations.Lever;
import incognito.teamcode.robot.component.slide.HorizontalSlide;
import incognito.teamcode.robot.component.slide.VerticalSlide;


public class Robot extends RobotCog {
    // ===== Instance Variables =====

    boolean cameraEnabled;

    // Inherit hardwareMap and telemetry from OpMode
    HardwareMap hardwareMap;
    Telemetry telemetry;

    // -- Components --
    public AutoCamera autoCamera;
    public Drivetrain drivetrain;
    public VerticalSlide verticalSlide;
    public HorizontalSlide horizontalSlide;
    public Claw verticalClaw;
    public Claw horizontalClaw;
    public Flipper verticalFlip;
    public ContinuousServo turret;
    public ContinuousServo hinge;
    public Lever lever;
    public DistanceSensor horizontalDistanceSensor;

    public Scheduler horizontalScheduler;
    public Scheduler verticalScheduler;

    public boolean isRetracting = false;
    public boolean isExtending = false;
    public boolean isDepositing = false;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, false);
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean cameraEnabled) {
        super(hardwareMap, telemetry);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.cameraEnabled = cameraEnabled;

        if (cameraEnabled)
            autoCamera = new AutoCamera(hardwareMap, telemetry);

        horizontalScheduler = new Scheduler();
        verticalScheduler = new Scheduler();
        configureDrivetrain();
        configureHorizontalSlide();
        configureVerticalSlide();
        configureClaws();
        configureArm();
    }

    private void configureDrivetrain() {
        DcMotorEx DT_frontRight_M = hardwareMap.get(DcMotorEx.class, "DT_frontRight_M");
        DcMotorEx DT_backRight_M = hardwareMap.get(DcMotorEx.class, "DT_backRight_M");
        DcMotorEx DT_frontLeft_M = hardwareMap.get(DcMotorEx.class, "DT_frontLeft_M");
        DcMotorEx DT_backLeft_M = hardwareMap.get(DcMotorEx.class, "DT_backLeft_M");
        drivetrain = new Drivetrain(hardwareMap, telemetry, DT_frontRight_M, DT_backRight_M, DT_frontLeft_M, DT_backLeft_M);
    }

    private void configureHorizontalSlide() {
        DcMotorEx HS_slide_M = hardwareMap.get(DcMotorEx.class, "HS_slide_M");
        horizontalSlide = new HorizontalSlide(hardwareMap, telemetry, HS_slide_M, HS_MIN_ENCODER, HS_MAX_ENCODER);
        horizontalSlide.addSetPosition(0);
        horizontalSlide.setPower(0);
        horizontalSlide.goToSetPosition(0);

        horizontalDistanceSensor = hardwareMap.get(DistanceSensor.class, "HS_DS");
    }

    private void configureVerticalSlide() {
        DcMotorEx VS_slideRight_M = hardwareMap.get(DcMotorEx.class, "VS_slideRight_M");
        DcMotorEx VS_slideLeft_M = hardwareMap.get(DcMotorEx.class, "VS_slideLeft_M");
        VS_slideRight_M.setDirection(DcMotorEx.Direction.REVERSE);
        DigitalChannel VS_limitSwitch_D = hardwareMap.get(DigitalChannel.class, "VS_limitSwitch_D");
        verticalSlide = new VerticalSlide(hardwareMap, telemetry, new DcMotorEx[]{VS_slideLeft_M, VS_slideRight_M}, VS_limitSwitch_D);
        verticalSlide.addSetPositionLengths(new double[]{0, VS_SP_LOW, VS_SP_MEDIUM, VS_SP_HIGH, VS_SP_AUTO});
        verticalSlide.setPower(0);
    }

    private void configureClaws() {
        Servo HS_claw_S = hardwareMap.get(Servo.class, "HS_claw_S");
        horizontalClaw = new Claw(hardwareMap, telemetry, HS_claw_S, HS_CLAW_OPEN, HS_CLAW_CLOSED);
        horizontalClaw.open();

        Servo VS_claw_S = hardwareMap.get(Servo.class, "VS_claw_S");
        verticalClaw = new Claw(hardwareMap, telemetry, VS_claw_S, VS_CLAW_OPEN, VS_CLAW_CLOSED);
        verticalClaw.close();

        Servo VS_flip_S = hardwareMap.get(Servo.class, "VS_flip_S");
        verticalFlip = new Flipper(hardwareMap, telemetry, VS_flip_S, VS_FLIP_DOWN, VS_FLIP_UP);
        verticalFlip.flipUp();
    }

    private void configureArm() {
        Servo HS_turret_S = hardwareMap.get(Servo.class, "HS_turret_S");
        turret = new ContinuousServo(hardwareMap, telemetry, HS_turret_S, HS_TURRET_START, HS_TURRET_MIN, HS_TURRET_MAX);
        turret.setOffsetFactor(HS_TURRET_SPEED);

        Servo HS_hinge_S = hardwareMap.get(Servo.class, "HS_hinge_S");
        hinge = new ContinuousServo(hardwareMap, telemetry, HS_hinge_S, HS_HINGE_IN);
        hinge.setOffsetFactor(HS_HINGE_SPEED);

        Servo HS_lever_S = hardwareMap.get(Servo.class, "HS_lever_S");
        telemetry.addLine("Creating lever");
        lever = new Lever(hardwareMap, telemetry, HS_lever_S,
                new double[]{HS_LEVER_IN,
                        HS_LEVER_MID,
                        HS_LEVER_FIFTH,
                        HS_LEVER_FOURTH,
                        HS_LEVER_THIRD,
                        HS_LEVER_SECOND,
                        HS_LEVER_OUT},
                HS_LEVER_OUT, HS_LEVER_IN);
        lever.goToSetPosition(Lever.HorizontalLeverPosition.IN);
        lever.setOffsetFactor(HS_LEVER_SPEED);
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
    }

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
            //lever.goToSetPosition(Lever.HorizontalLeverPosition.MID, false);
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
                        lever.goToSetPosition(Lever.HorizontalLeverPosition.IN, false);
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
        /* OLD
        // The angle of the lever in radians
        double leverAngle = (lever.getPosition() - HS_LEVER_FLAT) / HS_LEVER_TICKS_PER_RAD;

        // Subtract from 2PI to keep hinge parallel to ground
        double hingeParallel = leverAngle;

        // The position of the hinge in ticks
        double newHingePosition = hingeParallel * HS_HINGE_TICKS_PER_RAD + HS_HINGE_FLAT;

        hinge.setPosition(newHingePosition);
         */
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
        lever.goToSetPosition(Lever.HorizontalLeverPosition.OUT, true);
        levelHinge();
    }

    public boolean nearCone() {
        return nearCone(HS_SENSOR_DISTANCE_CM);
    }

    public boolean nearCone(double distance) {
        return horizontalDistanceSensor.getDistance(DistanceUnit.CM) < distance;
    }

    public void update() {
        verticalSlide.update();
        if (!isRetracting)
            horizontalSlide.update();
        horizontalScheduler.update();
        verticalScheduler.update();
    }
}