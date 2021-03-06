package org.firstinspires.ftc.teamcode.opmodes.autonomous.meet1time;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.westtorrancerobotics.lib.hardware.drive.MecanumController;


import org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber.Hook.BOTH;

public abstract class DummyThiccTesting extends TimedAuto{

    protected Robot bot;
    private MecanumController driveTrain;
    public void runOpMode() throws InterruptedException {
        bot = Robot.getInstance();
        bot.init(hardwareMap);
        backupGyro1 = hardwareMap.get(BNO055IMU.class, "imu1");
        backupGyro1.initialize(new BNO055IMU.Parameters());
        waitForStart();
        timeInches(10, -30);
        bot.foundationGrabber.setGrabbed(BOTH, true);
        sleep(2000);
        timeInches(0, 100);
        bot.foundationGrabber.setGrabbed(BOTH, false);
        sleep(2000);
        boolean saw = false;
        moveForTesting(0,2,0);
        sleep(3000);
        moveForTesting(0,0,0);
        sleep(3000);
        moveForTesting(2,0,0);
        sleep(3000);
        moveForTesting(2,2,0);
        sleep(3000);
        moveForTesting(0,0,0);
    }
    public void moveForTesting(double x, double y, double z) throws InterruptedException {
        bot.driveTrain.spinDrive(x, y, z);
    }

}
