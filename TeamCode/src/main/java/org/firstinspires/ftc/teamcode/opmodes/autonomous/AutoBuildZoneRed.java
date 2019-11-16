package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.westtorrancerobotics.lib.Angle;
import org.westtorrancerobotics.lib.Location;

import static org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber.Hook.LEFT;
import static org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber.Hook.RIGHT;
import static org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber.Hook.BOTH;

@Disabled
@Autonomous(name = "Red Build Zone", group = "none")
public class AutoBuildZoneRed extends LinearOpMode {
        @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = Robot.getInstance();
        bot.init(hardwareMap);
        bot.foundationGrabber.setGrabbed(LEFT,false);
        bot.foundationGrabber.setGrabbed(RIGHT,false);
        bot.lift.idle();
//        bot.stoneManipulator.stow();
        sleep(4000);
        bot.lift.zero();
//        bot.driveTrain.calibrateGyro();
//        while (bot.driveTrain.isCalibratingGyro()) {
//            sleep(1);
//        }
        bot.driveTrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.driveTrain.setLocation(new Location(24 * 3 - 9, 24 * 3 - 9,
                new Angle(180, Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING)));
        // start pushed into the corner, in the blue build site, facing toward the opponent driver station

        waitForStart();
        bot.runtime.reset();

        bot.camera.start();
        bot.driveTrain.spinDrive(0, 1, 0);
        while (bot.runtime.seconds() < 0.25) {
            bot.driveTrain.updateLocation();
            sleep(1);
        }
        bot.foundationGrabber.setGrabbed(LEFT,true);
        bot.foundationGrabber.setGrabbed(RIGHT,true);
        bot.driveTrain.spinDrive(0, -1, 0);
        while (bot.runtime.seconds() < 0.25) {
            bot.driveTrain.updateLocation();
            sleep(1);
        }
        bot.foundationGrabber.setGrabbed(LEFT,false);
        bot.foundationGrabber.setGrabbed(RIGHT,false);
        bot.driveTrain.spinDrive(-1, 0, 0);
        while (bot.driveTrain.getLocation().x > 0 && !bot.driveTrain.onBlueLine() && bot.runtime.seconds() < 5) {
            bot.driveTrain.updateLocation();
            sleep(1);
        }
        bot.driveTrain.spinDrive(0, 0, 0);
        bot.close();
    }
}

