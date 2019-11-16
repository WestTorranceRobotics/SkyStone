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
        timeInches(70, -70);
        bot.foundationGrabber.setGrabbed(BOTH, true);
        sleep(2000);
        timeInches(0, 100);
        bot.foundationGrabber.setGrabbed(BOTH, false);
        sleep(2000);
    }
}
