package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ElapsedTimeAuto extends LinearOpMode {

    private DcMotor left1;
    private DcMotor left2;
    private DcMotor right1;
    private DcMotor right2;

    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {

        left1  = hardwareMap.get(DcMotor.class, "left1");
        left2  = hardwareMap.get(DcMotor.class, "left2");
        right1 = hardwareMap.get(DcMotor.class, "right1");
        right2 = hardwareMap.get(DcMotor.class, "right2");

        right1.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.REVERSE);

        timer = new ElapsedTime();

        waitForStart();
        timer.reset();

        while (timer.seconds() < 2) {
            tankDrive(1, 1);
            sleep(1);
        }

        tankDrive(0, 0);
    }

    public void tankDrive(double left, double right) {
        left1 .setPower(left);
        left2 .setPower(left);
        right1.setPower(right);
        right2.setPower(right);
    }
}
