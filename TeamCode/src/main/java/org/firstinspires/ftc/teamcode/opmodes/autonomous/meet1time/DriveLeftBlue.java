package org.firstinspires.ftc.teamcode.opmodes.autonomous.meet1time;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Drive Left Blue Line")
public class DriveLeftBlue extends OpMode {

    Robot bot;
    boolean saw = false;

    @Override
    public void init() {
        bot = Robot.getInstance();
        bot.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (bot.driveTrain.onBlueLine() || saw) {
            bot.driveTrain.spinDrive(0, 0, 0);
        } else {
            bot.driveTrain.spinDrive(-0.5,0,0);
        }
    }
}
