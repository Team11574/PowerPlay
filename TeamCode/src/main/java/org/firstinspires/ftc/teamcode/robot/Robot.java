package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_HINGE_START;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_LEVER_IN;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_LEVER_MID;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_LEVER_OUT;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_MAX_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_MIN_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.HS_TURRET_START;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.SET_POSITION_THRESHOLD;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.VS_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.VS_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.VS_FLIP_DOWN;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.VS_FLIP_UP;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.VS_SP_HIGH;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.VS_SP_LOW;
import static org.firstinspires.ftc.teamcode.robot.component.slide.SlideConstants.VS_SP_MEDIUM;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.component.Component;
import org.firstinspires.ftc.teamcode.robot.component.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.component.camera.AutoCamera;
import org.firstinspires.ftc.teamcode.robot.component.claw.Claw;
import org.firstinspires.ftc.teamcode.robot.component.claw.Flipper;
import org.firstinspires.ftc.teamcode.robot.component.claw.Lever;
import org.firstinspires.ftc.teamcode.robot.component.servo.ContinuousServo;
import org.firstinspires.ftc.teamcode.robot.component.servo.SetServo;
import org.firstinspires.ftc.teamcode.robot.component.slide.HorizontalSlide;
import org.firstinspires.ftc.teamcode.robot.component.slide.VerticalSlide;
import org.firstinspires.ftc.teamcode.util.GamepadPlus;
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
    public Lever lever;

    private boolean isRetracting = false;
    

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
    }

    private void configureVerticalSlide() {
        DcMotorEx VS_slideRight_M = hardwareMap.get(DcMotorEx.class, "VS_slideRight_M");
        DcMotorEx VS_slideLeft_M = hardwareMap.get(DcMotorEx.class, "VS_slideLeft_M");
        VS_slideRight_M.setDirection(DcMotorEx.Direction.REVERSE);
        DigitalChannel VS_limitSwitch_D = hardwareMap.get(DigitalChannel.class, "VS_limitSwitch_D");
        verticalSlide = new VerticalSlide(hardwareMap, telemetry, new DcMotorEx[]{VS_slideLeft_M, VS_slideRight_M}, VS_limitSwitch_D);
        verticalSlide.addSetPositionLengths(new double[]{ VS_SP_LOW, VS_SP_MEDIUM, VS_SP_HIGH });
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
        turret = new ContinuousServo(hardwareMap, telemetry, HS_turret_S, HS_TURRET_START);
        turret.setOffsetFactor(0.01);

        Servo HS_hinge_S = hardwareMap.get(Servo.class, "HS_hinge_S");
        hinge = new ContinuousServo(hardwareMap, telemetry, HS_hinge_S, HS_HINGE_START);

        Servo HS_lever_S = hardwareMap.get(Servo.class, "HS_lever_S");
        lever = new Lever(hardwareMap, telemetry, HS_lever_S, HS_LEVER_IN, HS_LEVER_MID, HS_LEVER_OUT);
        lever.moveIn();
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

    public void retractArm() {
        if (!isRetracting) {
            isRetracting = true;
            horizontalSlide.goToSetPosition(0);
            turret.goToStartPosition();
            lever.moveMid();

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
            Scheduler.linearSchedule(
                    when -> horizontalSlide.atSetPosition(),
                    then -> lever.moveIn()
            );
            Scheduler.linearSchedule(
                    when -> true,
                    then -> horizontalClaw.open(),
                    500
            );
            Scheduler.linearSchedule(
                    when -> true,
                    then -> {
                        horizontalSlide.goToLastPosition();
                        lever.moveOut();
                        isRetracting = false;
                    },
                    500
            );

        } // else, do nothing. We don't want to double schedule movements.
    }

    public void update() {
        verticalSlide.update();
        horizontalSlide.update();
        Scheduler.update();
    }
}
