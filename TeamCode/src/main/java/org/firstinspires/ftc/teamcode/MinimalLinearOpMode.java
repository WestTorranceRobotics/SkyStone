package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Example Linear", group = "none")
public class MinimalLinearOpMode extends LinearOpMode {

    private Servo arm;
    private double position;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = hardwareMap.get(Servo.class, "arm");
        position = 0;
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.start && !gamepad1.back && position < 1) {
                position = position + 0.001;
            }
            if (gamepad1.back && !gamepad1.start && position > 1) {
                position = position - 0.001;
            }
            arm.setPosition(position);
        }
    }
}
