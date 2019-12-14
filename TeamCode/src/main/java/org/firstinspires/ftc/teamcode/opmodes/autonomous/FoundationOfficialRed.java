package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber;

@Autonomous(name = "FoundationOfficialRed", group = "none")
public class FoundationOfficialRed extends LinearOpMode {

    private Robot bot;
    private BNO055IMU backupGyro1;
    private DcMotor vEncoder;
    private DcMotor hEncoder;
    private double distanceV = 0;
    private double distanceH = 0;
    private final double DRIVE_SPEED = .5;
    private final double TURN_SPEED = .5;
    private final double TICKS_TO_INCHES = (2 * Math.PI) / 4096;
    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
    private double angleHeld = 0;
    private double currentAngle = 0;
    private double distanceEV = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        bot = Robot.getInstance();
        bot.init(hardwareMap);
        hEncoder = hardwareMap.get(DcMotorEx.class, "liftRight/odometerX");
        vEncoder = hardwareMap.get(DcMotorEx.class, "intakeRight/odometerRightY");
        backupGyro1 = hardwareMap.get(BNO055IMU.class, "imu1");
        backupGyro1.initialize(new BNO055IMU.Parameters());
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        resetEncoders();
        getData();
        resetEncoders();
        while (distanceEV < 16 && opModeIsActive()) {
            getData();
            forceAngle();
            leftBack.setPower(.5);
            leftFront.setPower(-.5);
            rightBack.setPower(-.5);
            rightFront.setPower(.5);
            getTelem();
        }
        bot.driveTrain.spinDrive(0, 0, 0);
        resetEncoders();
        while (distanceEV > -24 && opModeIsActive()) {
            getData();
            forceAngle();
            leftBack.setPower(-.5);
            leftFront.setPower(-.5);
            rightBack.setPower(-.5);
            rightFront.setPower(-.5);
            getTelem();
        }
        bot.driveTrain.spinDrive(0, 0, 0);
        resetEncoders();
        bot.foundationGrabber.setGrabbed(FoundationGrabber.Hook.BOTH, true);
        sleep(3000);
        while (distanceEV < 32 && opModeIsActive()) {
            getData();
            forceAngle();
            leftBack.setPower(.75);
            leftFront.setPower(.75);
            rightBack.setPower(.75);
            rightFront.setPower(.75);
            getTelem();
        }
        bot.driveTrain.spinDrive(0, 0, 0);
        resetEncoders();
        bot.foundationGrabber.setGrabbed(FoundationGrabber.Hook.BOTH, false);
        while (distanceEV > -55 && opModeIsActive()) {
            getData();
            forceAngle();
            leftBack.setPower(-.5);
            leftFront.setPower(.5);
            rightBack.setPower(.5);
            rightFront.setPower(-.5);
            getTelem();
        }
        bot.driveTrain.spinDrive(0, 0, 0);


        /*resetEncoders();

        sleep(3000);
        resetEncoders();
        while ( distanceEV > -34 && opModeIsActive()) {
            getData();
            forceAngle();
            bot.driveTrain.spinDrive(0, 1, 0);
            getTelem(); }




    }*/
    }


    /**
     * Call to reset Encoders
     */
    private void resetEncoders() {
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Call to update distance
     */
    private void getData() {
        currentAngle = backupGyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        distanceEV = leftBack.getCurrentPosition() * ((4 * Math.PI) / 560);
    }
    /**
     * Call to send telemetry
     * getData must be called for data to update
     */
    private void getTelem() {
        telemetry.addData("distanceEV", distanceEV);
        telemetry.addData("DEGREES", currentAngle);
        telemetry.update();

    } private void forceAngle () {
        while ((currentAngle > 5 || currentAngle < -5) && opModeIsActive()) {
            currentAngle = backupGyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            if (currentAngle > 5) {
                leftFront.setPower(.25);
                leftBack.setPower(.25);
                rightFront.setPower(-.25);
                rightBack.setPower(-.25);
            } else if
            (currentAngle < -5) {
                leftFront.setPower(-.25);
                leftBack.setPower(-.25);
                rightFront.setPower(.25);
                rightBack.setPower(.25);
            } else {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);

            }
        }
    }
}
