package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.component.camera.AutoCamera;

@Autonomous(name = "AUTO Camera Testing", group = "auto")
public class AutoCameraTesting extends LinearOpMode {

    double junctionDistance;

    @Override
    public void runOpMode() {
        MultipleTelemetry multiTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        AutoCamera camera = new AutoCamera(hardwareMap, multiTelemetry);
        Robot r = new Robot(hardwareMap, telemetry);

        camera.swapMode();
        //camera.terminateCamera();
        while (!isStarted()) {
            multiTelemetry.addData("Junction distance", camera.getJunctionDistance());
            multiTelemetry.update();
        }

        //camera.terminateCamera();

        waitForStart();


        while (!opModeIsActive()) {

            do {
                junctionDistance = camera.getJunctionDistance();
                multiTelemetry.addData("Junction distance", junctionDistance);
                double velY = Math.max(0, 1 / camera.getJunctionArea() - 0.1);
                double velX = 0;
                double theta = -0.005 * junctionDistance;

                double normalFactor = Math.max(Math.abs(velY) + Math.abs(velX) + Math.abs(theta), 1);
                double frontRight_Power = (velY - velX - theta) / normalFactor;
                double backRight_Power = (velY + velX - theta) / normalFactor;
                double frontLeft_Power = (velY + velX + theta) / normalFactor;
                double backLeft_Power = (velY - velX + theta) / normalFactor;

                r.drivetrain.setMotorPowers(frontLeft_Power, backLeft_Power, backRight_Power, frontRight_Power);
            } while (junctionDistance <= Double.POSITIVE_INFINITY && Math.abs(junctionDistance) >= 10);
            multiTelemetry.update();
        }

    }
}