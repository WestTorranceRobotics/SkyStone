package org.firstinspires.ftc.teamcode.opmodes.utility.calibration;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.Map;

@TeleOp(name = "Color Sensor Reader")
public class ColorCalibrate extends OpMode {

    private Object[] colors;
    private int index;
    private int pressed;

    @Override
    public void init() {
        colors = hardwareMap.colorSensor.entrySet().toArray();
        telemetry.addData("Number of color sensors found", colors.length);
        telemetry.addLine("Dpad up and down to switch color sensors.");
        telemetry.update();
        if (colors.length == 0) {
            throw new IllegalArgumentException("Needs at least one color sensor plugged in");
        }
        index = 0;
        pressed = 0;
    }

    @Override
    public void loop() {
        ColorSensor color = (ColorSensor) ((Map.Entry) colors[index]).getValue();
        String name = (String) ((Map.Entry) colors[index]).getKey();
        telemetry.addData("Color Sensor", name);
        telemetry.addData("red", color.red());
        telemetry.addData("blue", color.blue());
        telemetry.addData("green", color.green());
        telemetry.update();
        if (gamepad1.dpad_up && !gamepad1.dpad_down && pressed != 1) {
            pressed = 1;
            index++;
        }
        if (gamepad1.dpad_down && !gamepad1.dpad_up && pressed != -1) {
            pressed = -1;
            index += colors.length - 1;
        }
        if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
            pressed = 0;
        }
        index %= colors.length;
    }
}
