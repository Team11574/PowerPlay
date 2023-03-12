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
    double junctionArea;

    @Override
    public void runOpMode() {
        MultipleTelemetry multiTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        AutoCamera camera = new AutoCamera(hardwareMap, multiTelemetry);
        Robot r = new Robot(hardwareMap, telemetry);

        camera.swapMode();
        //camera.terminateCamera();
        while (!isStarted()) {
            multiTelemetry.addData("Junction distance", camera.getJunctionDistance());
            multiTelemetry.addData("Junction area", camera.getJunctionArea());
            multiTelemetry.update();
        }

        //camera.terminateCamera();

        waitForStart();

        double velY;
        double theta;
        do {
            junctionDistance = camera.getJunctionDistance();
            junctionArea = camera.getJunctionArea();
            multiTelemetry.addData("Junction distance", junctionDistance);
            multiTelemetry.addData("Junction area", junctionArea);
            double adjustedArea = 800 / junctionArea;
            double junctionMaxArea = 3000;
            multiTelemetry.addData("100/area", adjustedArea);
            if (junctionArea > junctionMaxArea) {
                velY = 0;
            } else {
                velY = Math.max(0, adjustedArea);
            }
            double velX = 0;
            if (Math.abs(junctionDistance) < 10) {
                theta = 0;
            } else {
                theta = -0.005 * junctionDistance;
            }


            double normalFactor = Math.max(Math.abs(velY) + Math.abs(velX) + Math.abs(theta), 1);
            double frontRight_Power = (velY - velX - theta) / normalFactor;
            double backRight_Power = (velY + velX - theta) / normalFactor;
            double frontLeft_Power = (velY + velX + theta) / normalFactor;
            double backLeft_Power = (velY - velX + theta) / normalFactor;

            r.drivetrain.setMotorPowers(frontLeft_Power, backLeft_Power, backRight_Power, frontRight_Power);
            multiTelemetry.update();
        } while (//(theta != 0 || velY != 0) &&
                    opModeIsActive());

    }
}