package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber;
import org.westtorrancerobotics.lib.Angle;
import org.westtorrancerobotics.lib.Location;
import org.westtorrancerobotics.lib.Point;

import static org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber.Hook.LEFT;
import static org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber.Hook.RIGHT;
import static org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber.Hook.BOTH;

@Disabled
//@Autonomous(name = "Blue Build Zone", group = "none")
public class AutoBuildZoneBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = Robot.getInstance();
        bot.init(hardwareMap);
        bot.foundationGrabber.setGrabbed(BOTH, false);

        bot.driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.driveTrain.setLocation(new Location(36, 63,
                new Angle(0, Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING)));

        waitForStart();
        bot.runtime.reset();

        bot.driveTrain.spinDrive(0, -1, 0);
        bot.runtime.reset();
        while (bot.runtime.seconds() < 0.25) {
            bot.driveTrain.updateLocation();
            if (!opModeIsActive()) {
                return;
            }
            telemetry.addData("Location", bot.driveTrain.getLocation());
            sleep(1);
        }
        bot.runtime.reset();
        while (bot.runtime.seconds() < 2 && (bot.driveTrain.getLocation().distance(new Point(40, 32)) < 1)) {
            bot.driveTrain.updateLocation();
            bot.driveTrain.spinDrive((40 + bot.driveTrain.getLocation().x) * 0.25, (32 + bot.driveTrain.getLocation().y) * 0.25,
                    bot.driveTrain.getLocation().direction.getValue(Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING) * 0.02);
            if (!opModeIsActive()) {
                return;
            }
            telemetry.addData("Location", bot.driveTrain.getLocation());
            sleep(1);
        }
        bot.driveTrain.spinDrive(0, 0, 0);
        bot.foundationGrabber.setGrabbed(BOTH, true);
        while (bot.runtime.seconds() < 0.5) {
            if (!opModeIsActive()) {
                return;
            }
            telemetry.addData("Location", bot.driveTrain.getLocation());
            sleep(1);
        }
        bot.runtime.reset();
        bot.driveTrain.spinDrive(0, -1, 0);
        while (bot.runtime.seconds() < 4) {
            if (!opModeIsActive()) {
                return;
            }
            telemetry.addData("Location", bot.driveTrain.getLocation());
            sleep(1);
        }
        bot.driveTrain.spinDrive(0, 0, 0);
    }

}
