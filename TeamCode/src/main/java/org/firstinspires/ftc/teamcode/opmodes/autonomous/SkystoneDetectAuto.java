package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

public class SkystoneDetectAuto extends LinearOpMode {
    Robot bot;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    private void strafe(double xPow, double yPow) {
        double gyro = gyro();
        bot.driveTrain.spinDrive(x / max, y / max, Math.sqrt(Math.abs(gyro)) / 9 * Math.signum(gyro));
    }
}
