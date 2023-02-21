package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.components.slides.SlideConstants.HS_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.robot.components.slides.SlideConstants.HS_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.robot.components.slides.SlideConstants.HS_HINGE_START;
import static org.firstinspires.ftc.teamcode.robot.components.slides.SlideConstants.HS_TURRET_START;
import static org.firstinspires.ftc.teamcode.robot.components.slides.SlideConstants.VS_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.robot.components.slides.SlideConstants.VS_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.robot.components.slides.SlideConstants.VS_SP_HIGH;
import static org.firstinspires.ftc.teamcode.robot.components.slides.SlideConstants.VS_SP_LOW;
import static org.firstinspires.ftc.teamcode.robot.components.slides.SlideConstants.VS_SP_MEDIUM;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.components.Component;
import org.firstinspires.ftc.teamcode.robot.components.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.components.camera.Camera;
import org.firstinspires.ftc.teamcode.robot.components.claws.Claw;
import org.firstinspires.ftc.teamcode.robot.components.claws.ContinuousServo;
import org.firstinspires.ftc.teamcode.robot.components.slides.HorizontalSlide;
import org.firstinspires.ftc.teamcode.robot.components.slides.VerticalSlide;

public class Robot extends Component {
    // ===== Instance Variables =====

    boolean cameraEnabled;

    // Inherit hardwareMap and telemetry from OpMode
    HardwareMap hardwareMap;
    Telemetry telemetry;

    // -- Components --
    private Camera camera;
    private Drivetrain drivetrain;
    private VerticalSlide verticalSlide;
    private HorizontalSlide horizontalSlide;
    private Claw verticalClaw;
    private Claw horizontalClaw;
    private ContinuousServo turret;
    private ContinuousServo hinge;
    //private MotorGroup lever;
    

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, false);
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean cameraEnabled) {
        super(hardwareMap, telemetry);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.cameraEnabled = cameraEnabled;

        if (cameraEnabled)
            camera = new Camera(hardwareMap, telemetry);

        configureDrivetrain();
        configureHorizontalSlide();
        configureVerticalSlide();
        configureClaws();
        configureArm();
    }

    public void configureDrivetrain() {
        DcMotorEx DT_frontRight_M = hardwareMap.get(DcMotorEx.class, "DT_frontRight_M");
        DcMotorEx DT_backRight_M = hardwareMap.get(DcMotorEx.class, "DT_backRight_M");
        DcMotorEx DT_frontLeft_M = hardwareMap.get(DcMotorEx.class, "DT_frontLeft_M");
        DcMotorEx DT_backLeft_M = hardwareMap.get(DcMotorEx.class, "DT_backLeft_M");
        drivetrain = new Drivetrain(hardwareMap, telemetry, DT_frontRight_M, DT_backRight_M, DT_frontLeft_M, DT_backLeft_M);
    }

    public void configureHorizontalSlide() {
        DcMotorEx HS_slide_M = hardwareMap.get(DcMotorEx.class, "HS_slide_M");
        horizontalSlide = new HorizontalSlide(hardwareMap, telemetry, HS_slide_M);
    }

    public void configureVerticalSlide() {
        DcMotorEx VS_slideRight_M = hardwareMap.get(DcMotorEx.class, "HS_slideRight_M");
        DcMotorEx VS_slideLeft_M = hardwareMap.get(DcMotorEx.class, "HS_slideLeft_M");
        DigitalChannel VS_limitSwitch_D = hardwareMap.get(DigitalChannel.class, "VS_limitSwitch_D");
        verticalSlide = new VerticalSlide(hardwareMap, telemetry, new DcMotorEx[]{VS_slideLeft_M, VS_slideRight_M}, VS_limitSwitch_D);

        verticalSlide.addSetPositionLengths(new double[]{VS_SP_LOW, VS_SP_MEDIUM, VS_SP_HIGH});
    }

    public void configureClaws() {
        Servo HS_claw_S = hardwareMap.get(Servo.class, "HS_claw_S");
        horizontalClaw = new Claw(hardwareMap, telemetry, HS_claw_S, HS_CLAW_OPEN, HS_CLAW_CLOSED);
        horizontalClaw.close();

        Servo VS_claw_S = hardwareMap.get(Servo.class, "VS_claw_S");
        verticalClaw = new Claw(hardwareMap, telemetry, VS_claw_S, VS_CLAW_OPEN, VS_CLAW_CLOSED);
        verticalClaw.close();
    }

    public void configureArm() {
        Servo HS_turret_S = hardwareMap.get(Servo.class, "HS_turret_S");
        turret = new ContinuousServo(hardwareMap, telemetry, HS_turret_S, HS_TURRET_START);
        turret.setOffsetFactor(0.01);

        Servo HS_hinge_S = hardwareMap.get(Servo.class, "HS_hinge_S");
        hinge = new ContinuousServo(hardwareMap, telemetry, HS_hinge_S, HS_HINGE_START);
        
        DcMotorEx HS_lever_M = hardwareMap.get(DcMotorEx.class, "HS_lever_M");
        //lever = new ;
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }
    public HorizontalSlide getHorizontalSlide() { return horizontalSlide; };
    public VerticalSlide getVerticalSlide() { return verticalSlide; };

    public int getParkingSpot() {
        if (cameraEnabled)
            return camera.getParkingSpot();
        return -1;
    }

    public void update() {
        verticalSlide.update();
    }
}
