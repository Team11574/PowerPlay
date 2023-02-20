package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.components.Component;
import org.firstinspires.ftc.teamcode.robot.components.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.components.camera.Camera;

public class Robot extends Component {
    // ===== Instance Variables =====

    // Inherit hardwareMap and telemetry from OpMode
    HardwareMap hardwareMap;
    Telemetry telemetry;

    // -- Components --
    private Drivetrain drivetrain;
    private Camera camera;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        camera = new Camera(hardwareMap, telemetry);
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public int getParkingSpot() {
        return camera.getParkingSpot();
    }
}
