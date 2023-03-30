package incognito.teamcode.opmodes.testing;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Range Sensor Test", group = "testing")
public class RangeSensor extends OpMode {
    ModernRoboticsI2cRangeSensor range;
    DigitalChannelImpl ultraOut;
    DigitalChannelImpl ultraIn;
    UltrasonicSensor rangeSensor;

    @Override
    public void init() {
        /*range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        ultraOut = hardwareMap.get(DigitalChannelImpl.class, "ultraOut");
        ultraIn = hardwareMap.get(DigitalChannelImpl.class, "ultraIn");
        ultraOut.setMode(DigitalChannel.Mode.OUTPUT);
        ultraIn.setMode(DigitalChannel.Mode.INPUT);
        ultraOut.setState(true);*/
        rangeSensor = hardwareMap.get(UltrasonicSensor.class, "ultraRangeSensor");


    }

    @Override
    public void loop() {
        /*telemetry.addData("Range value", range.getDistance(DistanceUnit.MM));
        telemetry.addData("Analog value", ultraIn.getState());*/
    }
}