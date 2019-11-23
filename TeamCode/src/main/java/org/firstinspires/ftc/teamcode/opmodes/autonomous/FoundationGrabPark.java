package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.meet1time.TimedAuto;

import static org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber.Hook.BOTH;

@Autonomous(name = "CHANCE IS A MEERKAT", group = "none")
public class FoundationGrabPark extends LinearOpMode {

    private Robot bot;
    private BNO055IMU backupGyro1;
    private DcMotor vEncoder;
    private DcMotor hEncoder;
    private double distanceV = 0;
    private double distanceH = 0;
    private final double DRIVE_SPEED = .5;
    private final double TURN_SPEED = .5;
    private final double TICKS_TO_INCHES = (2 * Math.PI) / 4096;
    @Override
    public void runOpMode() throws InterruptedException {
        bot = Robot.getInstance();
        bot.init(hardwareMap);
        hEncoder = hardwareMap.get(DcMotorEx.class, "liftRight/odometerX");
        vEncoder = hardwareMap.get(DcMotorEx.class, "intakeRight/odometerRightY");
        backupGyro1 = hardwareMap.get(BNO055IMU.class, "imu1");
        backupGyro1.initialize(new BNO055IMU.Parameters());

        waitForStart();
        resetEncoders();
        while (distanceV < 13 && opModeIsActive()) {
            getDistance();
            bot.driveTrain.spinDrive(0, -DRIVE_SPEED, 0);
            getTelem();
        }
        resetEncoders();
        while (backupGyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 31.799 && opModeIsActive()) {
            getDistance();
            bot.driveTrain.spinDrive(0, 0, -TURN_SPEED);
            getTelem();
        }
        resetEncoders();
        while (distanceV < 17 && opModeIsActive()) {
            getDistance();
            bot.driveTrain.spinDrive(0, -DRIVE_SPEED, 0);
            getTelem();
        }
        resetEncoders();
        while (backupGyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 98.4872 && opModeIsActive()) {
            getDistance();
            bot.driveTrain.spinDrive(0, 0, -TURN_SPEED);
            getTelem();
        }
        resetEncoders();
        while (distanceV < 6.445 && opModeIsActive()) {
            getDistance();
            bot.driveTrain.spinDrive(0, DRIVE_SPEED, 0);
            getTelem();
        }
        resetEncoders();
        while (distanceH < 10 && opModeIsActive()) {
            getDistance();
            bot.driveTrain.spinDrive(-DRIVE_SPEED, 0, 0);
        }
        resetEncoders();
        while (backupGyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 89.668 && opModeIsActive()) {
            getDistance();
            bot.driveTrain.spinDrive(0, 0, -TURN_SPEED);
            getTelem();
        }
        resetEncoders();
        while (distanceH < 20.24 && opModeIsActive()) {
            getDistance();
            bot.driveTrain.spinDrive(DRIVE_SPEED, 0, 0);
            getTelem();
        }
        resetEncoders();
        while (distanceV < 41.707 && opModeIsActive()) {
            getDistance();
            bot.driveTrain.spinDrive(0, -DRIVE_SPEED, 0);
            getTelem();
        }
        resetEncoders();


    }

    private void resetEncoders() {
        vEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void getDistance() {
        distanceV = vEncoder.getCurrentPosition() * TICKS_TO_INCHES;
        distanceH = hEncoder.getCurrentPosition() * TICKS_TO_INCHES;
    } private void getTelem() {
        telemetry.addData("distanceH", distanceH);
        telemetry.addData("distanceV", distanceV);
        telemetry.addData("DEGREES", backupGyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();
    }
}
