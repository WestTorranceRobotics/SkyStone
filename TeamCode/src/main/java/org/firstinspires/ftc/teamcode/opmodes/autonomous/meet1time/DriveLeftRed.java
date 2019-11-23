package org.firstinspires.ftc.teamcode.opmodes.autonomous.meet1time;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Drive Left Red Line")
public class DriveLeftRed extends OpMode {

    Robot bot;
    RevColorSensorV3 color;
    boolean saw = false;

    @Override
    public void init() {
        color = hardwareMap.get(RevColorSensorV3.class, "lineColor");
        bot = Robot.getInstance();
        bot.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (bot.driveTrain.onRedLine() || saw) {
            bot.driveTrain.spinDrive(0, 0, 0);
            saw = true;
        } else {
            bot.driveTrain.spinDrive(-0.5,0,0);
        }
        telemetry.addData("red", color.red());
        telemetry.addData("blue", color.blue());
    }
}
