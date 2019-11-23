package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber.SeenObject;
import org.westtorrancerobotics.lib.Angle;

import java.util.function.DoubleSupplier;

import static org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber.Hook.EITHER;
import static org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber.Hook.LEFT;
import static org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber.Hook.RIGHT;
import static org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber.SeenObject.SKYSTONE;
import static org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber.SeenObject.STONE;

@Autonomous(name = "Skystone Boy", group = "none")
public class SkystoneDetectAuto extends LinearOpMode {

    Robot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = Robot.getInstance();
        bot.init(hardwareMap);
        waitForStart();
        Angle dir = bot.driveTrain.gyro();
        while (opModeIsActive()) {
            bot.driveTrain.spinDrive(0, -0.5, 0);
        }
       /* while(opModeIsActive() && bot.foundationGrabber.getDistancePredicated(EITHER, (dst) -> dst > 3)) {
            bot.driveTrain.spinDrive(0, -0.5, 0);
            telemetry.addData("dstLeft", bot.foundationGrabber.getDistance(LEFT));
            telemetry.addData("dstRight", bot.foundationGrabber.getDistance(RIGHT));
            telemetry.addData("dirTarg", dir);
            telemetry.addData("dirActu", bot.driveTrain.gyro());
            telemetry.update();
        }*/
        telemetry.clearAll();
        telemetry.update();
        SeenObject left = bot.foundationGrabber.getView(LEFT, bot.foundationGrabber.getDistance(LEFT));
        SeenObject right = bot.foundationGrabber.getView(RIGHT, bot.foundationGrabber.getDistance(RIGHT));
        DoubleSupplier dst = () -> left == STONE ? bot.foundationGrabber.getDistance(LEFT)
                : right == STONE ? bot.foundationGrabber.getDistance(RIGHT)
                : (bot.foundationGrabber.getDistance(LEFT) + bot.foundationGrabber.getDistance(RIGHT) / 2);
        sleep(1000);
        while(opModeIsActive() && !bot.foundationGrabber.sideTouchingFoundation()) {
            telemetry.addData("dst", dst.getAsDouble());
            telemetry.update();
            bot.driveTrain.holdDirTranslate(0, -dst.getAsDouble() / 8, dir);
        }
        telemetry.addData("left obj", left);
        telemetry.addData("right obj", right);
        String target = "not found";
        if (left == SKYSTONE && right != SKYSTONE) {
            target = "left";
        }
        if (right == SKYSTONE && left != SKYSTONE) {
            target = "right";
        }
        if (right == STONE && left == STONE) {
            target = "center";
        }
        telemetry.addData("target", target);
        telemetry.update();
        if (opModeIsActive()) {
            sleep(10000);
        }
    }
}
