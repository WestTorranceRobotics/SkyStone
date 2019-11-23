package org.firstinspires.ftc.teamcode.opmodes.utility.calibration;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import java.util.Map;

@TeleOp(name = "Rev v3 Skystone Calibrator")
public class DistanceCalibrate extends OpMode {

    private RevColorSensorV3 color;
    private int index;
    private int pressed;

    @Override
    public void init() {
        color = hardwareMap.get(RevColorSensorV3.class, "ssColorLeft");
    }

    @Override
    public void loop() {
        telemetry.addData("Red/Blue", color.red() / (double) color.blue());
        telemetry.addData("Green/Blue", color.green() / (double) color.blue());
        telemetry.addData("Raw optical", color.rawOptical());
    }
}
