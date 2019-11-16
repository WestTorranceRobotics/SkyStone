package org.firstinspires.ftc.teamcode.opmodes.utility;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "color calibrate")
public class ColorCalibrate extends OpMode {

    RevColorSensorV3 color;

    @Override
    public void init() {
        color = hardwareMap.get(RevColorSensorV3.class, "lineColor");
    }

    @Override
    public void loop() {
        telemetry.addData("red", color.red());
        telemetry.addData("blue", color.blue());
    }
}
