package org.firstinspires.ftc.teamcode.opmodes.utility;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "lltest", group = "none")
public class LogicLevelTester extends OpMode {

    private ModernRoboticsI2cGyro gyro;

    @Override
    public void init() {
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
    }

    @Override
    public void loop() {
        telemetry.addData("gyro", gyro.getIntegratedZValue());
        telemetry.update();
    }
}
