package org.firstinspires.ftc.teamcode.opmodes.utility.calibration;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import java.util.Map;

@TeleOp(name = "Distance Sensor Reader")
public class DistanceCalibrate extends OpMode {

    private Object[] distances;
    private int index;
    private int pressed;

    @Override
    public void init() {
        distances = hardwareMap.opticalDistanceSensor.entrySet().toArray();
        telemetry.addData("Number of distance sensor found", distances.length);
        telemetry.addLine("Dpad up and down to switch color sensors.");
        telemetry.update();
        if (distances.length == 0) {
            throw new IllegalArgumentException("Needs at least one distnace sensor plugged in");
        }
        index = 0;
        pressed = 0;
    }

    @Override
    public void loop() {
        OpticalDistanceSensor distanceSensor = (OpticalDistanceSensor) ((Map.Entry) distances[index]).getValue();
        String name = (String) ((Map.Entry) distances[index]).getKey();
        telemetry.addData("Distance Sensor", name);
        telemetry.addData("Raw Light Value", distanceSensor.getRawLightDetected());
        telemetry.update();
        if (gamepad1.dpad_up && !gamepad1.dpad_down && pressed != 1) {
            pressed = 1;
            index++;
        }
        if (gamepad1.dpad_down && !gamepad1.dpad_up && pressed != -1) {
            pressed = -1;
            index += distances.length - 1;
        }
        if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
            pressed = 0;
        }
        index %= distances.length;
    }
}
