package org.firstinspires.ftc.teamcode.opmodes.autonomous.meet1time;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber.Hook.BOTH;

@Autonomous(name = "DEPOT AUTO", group = "none")
public class DEPOT extends TimedAuto {
    private Robot bot;
    private BNO055IMU backupGyro1;
    private DcMotor vEncoder;
    private DcMotor hEncoder;
    private double distanceV = 0;
    private double distanceH = 0;
    private final double DRIVE_SPEED = .75;
    private final double TURN_SPEED = .5;
    private final double TICKS_TO_INCHES = (2 * Math.PI) / 4096;
    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;

    @Override
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
        hEncoder = hardwareMap.get(DcMotorEx.class, "liftRight/odometerX");
        vEncoder = hardwareMap.get(DcMotorEx.class, "intakeRight/odometerRightY");
        backupGyro1 = hardwareMap.get(BNO055IMU.class, "imu1");
        backupGyro1.initialize(new BNO055IMU.Parameters());
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        waitForStart();
        resetEncoders();
        /*while (opModeIsActive()) {
            getDistance();
            if (gamepad2.a) {resetEncoders();}
            getTelem();

        }*/
        while ( opModeIsActive()) {
            getDistance();
            bot.driveTrain.spinDrive(DRIVE_SPEED, 0, 0);
            getTelem();
        }
        resetEncoders();
        while (distanceV < 31.92 && opModeIsActive()) {
            getDistance();
            bot.driveTrain.spinDrive(0, -DRIVE_SPEED, 0);
            getTelem();
        }
        resetEncoders();




        sleep(10000);
        boolean saw = false;
        while (opModeIsActive() && !saw) {
            if (bot.driveTrain.onBlueLine() || saw) {
                bot.driveTrain.spinDrive(0, 0, 0);
                saw = true;
            } else {
                bot.driveTrain.spinDrive(0.5, 0, 0);
            }
        }
        while (opModeIsActive() && !saw) {
            if (bot.driveTrain.onBlueLine() || saw) {
                bot.driveTrain.spinDrive(0, 0, 0);
                saw = true;
            } else {
                bot.driveTrain.spinDrive(-0.5, 0, 0);
            }


        }
    }
    private void resetEncoders () {
        vEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void getDistance () {
        distanceV = vEncoder.getCurrentPosition() * TICKS_TO_INCHES;
        distanceH = hEncoder.getCurrentPosition() * TICKS_TO_INCHES;
    }
    private void getTelem () {
        telemetry.addData("distanceH", distanceH);
        telemetry.addData("distanceV", distanceV);
        telemetry.addData("DEGREES", backupGyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();
    }
}


