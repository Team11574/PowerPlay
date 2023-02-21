package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.components.slides.SlideConstants.VS_SP_HIGH;
import static org.firstinspires.ftc.teamcode.robot.components.slides.SlideConstants.VS_SP_LOW;
import static org.firstinspires.ftc.teamcode.robot.components.slides.SlideConstants.VS_SP_MEDIUM;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.components.Component;
import org.firstinspires.ftc.teamcode.robot.components.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.components.camera.Camera;
import org.firstinspires.ftc.teamcode.robot.components.slides.HorizontalSlide;
import org.firstinspires.ftc.teamcode.robot.components.slides.Slide;
import org.firstinspires.ftc.teamcode.robot.components.slides.VerticalSlide;

public class Robot extends Component {
    // ===== Instance Variables =====

    boolean cameraEnabled;

    // Inherit hardwareMap and telemetry from OpMode
    HardwareMap hardwareMap;
    Telemetry telemetry;

    // -- Components --
    private Drivetrain drivetrain;
    private Camera camera;
    private VerticalSlide verticalSlide;
    private HorizontalSlide horizontalSlide;

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
        verticalSlide = new VerticalSlide(hardwareMap, telemetry, new DcMotorEx[]{VS_slideLeft_M, VS_slideRight_M});

        verticalSlide.addSetPositionLengths(new double[]{VS_SP_LOW, VS_SP_MEDIUM, VS_SP_HIGH});
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
}
