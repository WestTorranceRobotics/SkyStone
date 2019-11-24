package org.firstinspires.ftc.teamcode.opmodes.autonomous.meet1time;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robot;

public abstract class TimedAuto extends LinearOpMode {
    protected Robot bot;
    protected BNO055IMU backupGyro1;
    public void timeInches(double x, double y) {
        double time = Math.hypot(x, y) / 35;
        double max = Math.max(Math.abs(x), Math.abs(y));
        bot.runtime.reset();
        while (bot.runtime.seconds() < time) {
            double gyro = gyro();
            bot.driveTrain.spinDrive(x / max, y / max, Math.sqrt(Math.abs(gyro)) / 9 * Math.signum(gyro));
            if (!opModeIsActive()) {
                return;
            }
            sleep(1);
        }
        bot.driveTrain.spinDrive(0, 0, 0);
    }

    private double gyro() {
        return backupGyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
