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
        double initGyro = bot.driveTrain.gyro();
        while (bot.runtime.seconds() < 0.7) {
            telemetry.addData("Status", "Aligning with foundation");
            telemetry.update();
            bot.driveTrain.forceTranslate(-1,0, initGyro);
            if (!opModeIsActive()) {
                return;
            }
        }
        while (bot.runtime.seconds() < 0.7) {
            telemetry.addData("Status", "Aligning with foundation");
            telemetry.update();
            bot.driveTrain.forceTranslate(-1,0, initGyro);
            if (!opModeIsActive()) {
                return;
            }
        }
    }
}
