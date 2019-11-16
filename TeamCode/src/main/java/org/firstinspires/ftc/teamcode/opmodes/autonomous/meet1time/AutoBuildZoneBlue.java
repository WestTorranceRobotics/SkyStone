package org.firstinspires.ftc.teamcode.opmodes.autonomous.meet1time;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber.Hook.BOTH;

@Autonomous(name = "Blue Build Zone", group = "none")
public class AutoBuildZoneBlue extends TimedAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        bot = Robot.getInstance();
        bot.init(hardwareMap);
        backupGyro1 = hardwareMap.get(BNO055IMU.class, "imu1");
        backupGyro1.initialize(new BNO055IMU.Parameters());
        waitForStart();
        timeInches(0, -30);
        bot.foundationGrabber.setGrabbed(BOTH, true);
        sleep(2000);
        timeInches(0, 100);
        bot.foundationGrabber.setGrabbed(BOTH, false);
        sleep(2000);
        boolean saw = false;
        while (opModeIsActive() && !saw) {
            if (bot.driveTrain.onBlueLine() || saw) {
                bot.driveTrain.spinDrive(0, 0, 0);
                saw = true;
            } else {
                bot.driveTrain.spinDrive(-0.5,0,0);
            }
        }
    }
}
