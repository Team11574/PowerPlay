package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_HINGE_SPEED;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_HINGE_START;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_LEVER_IN;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_LEVER_MID;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_LEVER_OUT;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_LEVER_SPEED;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_MAX_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_MIN_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_TURRET_MAX;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_TURRET_MIN;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_TURRET_SPEED;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_TURRET_START;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.VS_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.VS_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.VS_FLIP_DOWN;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.VS_FLIP_UP;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.VS_SP_AUTO;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.VS_SP_HIGH;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.VS_SP_LOW;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.VS_SP_MEDIUM;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.component.Component;
import org.firstinspires.ftc.teamcode.robot.component.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.component.camera.AutoCamera;
import org.firstinspires.ftc.teamcode.robot.component.claw.Claw;
import org.firstinspires.ftc.teamcode.robot.component.claw.Flipper;
import org.firstinspires.ftc.teamcode.robot.component.servo.ContinuousServo;
import org.firstinspires.ftc.teamcode.robot.component.slide.HorizontalSlide;
import org.firstinspires.ftc.teamcode.robot.component.slide.VerticalSlide;
import org.firstinspires.ftc.teamcode.util.runnable.Scheduler;

public class Robot extends Component {
    // ===== Instance Variables =====

    boolean cameraEnabled;

    // Inherit hardwareMap and telemetry from OpMode
    HardwareMap hardwareMap;
    Telemetry telemetry;

    // -- Components --
    private AutoCamera autoCamera;
    public Drivetrain drivetrain;
    public VerticalSlide verticalSlide;
    public HorizontalSlide horizontalSlide;
    public Claw verticalClaw;
    public Claw horizontalClaw;
    public Flipper verticalFlip;
    public ContinuousServo turret;
    public ContinuousServo hinge;
    public ContinuousServo lever;

    public Scheduler horizontalScheduler;
    public Scheduler verticalScheduler;

    public boolean isRetracting = false;
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
        hinge = new ContinuousServo(hardwareMap, telemetry, HS_hinge_S, HS_HINGE_START);
        hinge.setOffsetFactor(HS_HINGE_SPEED);

        Servo HS_lever_S = hardwareMap.get(Servo.class, "HS_lever_S");
        telemetry.addLine("Creating lever");
        lever = new ContinuousServo(hardwareMap, telemetry, HS_lever_S, new double[]{HS_LEVER_IN, HS_LEVER_MID, HS_LEVER_OUT}, HS_LEVER_OUT, HS_LEVER_IN);
        lever.goToSetPosition(0);
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
        lever.offsetPosition(amount);
    }

    public void moveTurret(double amount) {
        turret.offsetPosition(amount);
    }

    public void retractArm() {
        if (!isRetracting) {
            isRetracting = true;
            //horizontalSlide.goToSetPosition(0);
            turret.goToSetPosition(0);
            lever.goToSetPosition(1);

            /* TODO: Determine if we can find a position where lever is positioned so that
                the cone rests just above the lip of the enclosure.
                If we can, this should work alright. It'll simply move the lever arm before
                retracting the horizontal slides and drop the cone once those horizontal
                slides reach 0.
                If we can't then we have problems. Servos don't have
                getCurrentPosition/getTargetPosition functions, they just have a
                getPosition which outputs what position the servo should be at. So, we don't
                have a great way of linearly scheduling the cone to drop once the lever
                servo is in position, since we have no way of actually knowing if the lever
                servo is in position or not.
             */
            horizontalScheduler.linearSchedule(
                    when -> horizontalSlide.goToPositionConstant(0),
                    // Lever in
                    then -> lever.goToSetPosition(0)
            );
            horizontalScheduler.linearSchedule(
                    when -> true,
                    then -> horizontalClaw.open(),
                    500
            );
            horizontalScheduler.linearSchedule(
                    when -> true,
                    then -> {
                        horizontalSlide.goToLastPosition();
                        // Lever out
                        lever.goToSetPosition(2);
                        isRetracting = false;
                    },
                    500
            );

        } // else, do nothing. We don't want to double schedule movements.
    }

    public void waitForRetract() {
        // NOT ASYNC
        while (horizontalScheduler.hasLinearQueries() || horizontalScheduler.hasGlobalQueries()) {
            horizontalScheduler.update();
        }
    }

    public void waitForDeposit() {
        while(verticalScheduler.hasLinearQueries() || verticalScheduler.hasGlobalQueries()) {
            verticalScheduler.update();
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
                        isDepositing = false;
                    },
                    500
            );
        }
    }

    public void update() {
        verticalSlide.update();
        if (!isRetracting)
            horizontalSlide.update();
        horizontalScheduler.update();
        verticalScheduler.update();
    }
}
