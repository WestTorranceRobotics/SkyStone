package org.firstinspires.ftc.teamcode.opmodes.autonomous.meet1time;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "get it boys", group = "none")
public class FoundationGet extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot;
        bot = Robot.getInstance();
        bot.init(hardwareMap);
        waitForStart();
        bot.runtime.reset();
        while (bot.runtime.seconds() < 1.5) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            bot.driveTrain.forceTranslate(-0.5,0);
            if (!opModeIsActive()) {
                return;
            }
        }
        bot.driveTrain.spinDrive(0,0,0);
    }
}
