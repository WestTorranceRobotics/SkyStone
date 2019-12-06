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

@Autonomous(name = "CHANCE IS A MEERKAT", group = "none")
/**
 *
 * Adjust turn / Drive speed to taste
 *
 * Speed should not impact distance traveled by a significant amount
 *
 */
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
    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
    private double angleHeld = 0;
    private double currentAngle = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        bot = Robot.getInstance();
        bot.init(hardwareMap);
        hEncoder = hardwareMap.get(DcMotorEx.class, "liftRight/odometerX");
        vEncoder = hardwareMap.get(DcMotorEx.class, "intakeRight/odometerRightY");
        backupGyro1 = hardwareMap.get(BNO055IMU.class, "imu1");
        backupGyro1.initialize(new BNO055IMU.Parameters());
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        waitForStart();
        resetEncoders();
        getData();
        angleHeld = currentAngle;
        while (opModeIsActive()) {
            getData();
            if (currentAngle != angleHeld + .5 || currentAngle != angleHeld - .5 && currentAngle > 0)
            {
               bot.driveTrain.spinDrive(0, 0, TURN_SPEED);
            }
            else if (currentAngle != angleHeld + .5 || currentAngle != angleHeld - .5 && currentAngle < 0)
                {
                    bot.driveTrain.spinDrive(0, 0, -TURN_SPEED);
                } else {
                bot.driveTrain.spinDrive(DRIVE_SPEED, 0, 0);
            }
        }


    }

    /**
     * Call to reset Encoders
     */
    private void resetEncoders() {
        vEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Call to update distance
     */
    private void getData() {
        distanceV = vEncoder.getCurrentPosition() * TICKS_TO_INCHES;
        distanceH = hEncoder.getCurrentPosition() * TICKS_TO_INCHES;
        currentAngle = backupGyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

    }
    /**
     * Call to send telemetry
     * getData must be called for data to update
     */
    private void getTelem() {
        telemetry.addData("distanceH", distanceH);
        telemetry.addData("distanceV", distanceV);
        telemetry.addData("DEGREES", currentAngle);
        telemetry.update();
    }
    /**
     * Call for turning in auto (takes angle in degrees)
     * NOT WORKING AT THIS MOMENT
     */
    private void actionTurn (double angle) {
        double speed = 0;
        speed = (angle < 0) ? TURN_SPEED : (angle > 0) ? -TURN_SPEED: -TURN_SPEED;
        if (angle > 0) {} else if (angle < 0) {}  else {}
        while ( angle < backupGyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle && opModeIsActive()) {
            getData();
            bot.driveTrain.spinDrive(0, 0, 0);
            getTelem();
        }
        resetEncoders();
    }
    /**
     * Call for moving horizontally in auto (takes inches)
     */
    private void actionHorizontal (double inch) {
        double speed = 0;
        speed = (inch > 0) ? DRIVE_SPEED : (inch < 0) ? -DRIVE_SPEED: DRIVE_SPEED;
        if (inch > 0)
        {
            while ( distanceV < inch && opModeIsActive()) {
                getData();
                bot.driveTrain.spinDrive(speed, 0, 0);
                getTelem(); }
        }
        else if (inch < 0)
        {
            while ( distanceV > inch && opModeIsActive()) {
                getData();
                bot.driveTrain.spinDrive(speed, 0, 0);
                getTelem(); }
        }
        else
        {
            while ( distanceV > inch && opModeIsActive()) {
                getData();
                bot.driveTrain.spinDrive(speed, 0, 0);
                getTelem(); }
        }
        resetEncoders();
    }
    /**
     * Call for moving vertically in auto (takes inches)
     */
    private void actionVertical (double inch) {
        double speed = 0;
        speed = (inch > 0) ? DRIVE_SPEED : (inch < 0) ? -DRIVE_SPEED: DRIVE_SPEED;
        if (inch > 0)
        {
            while ( distanceV < inch && opModeIsActive()) {
            getData();
            bot.driveTrain.spinDrive(speed, 0, 0);
            getTelem(); }
        }
        else if (inch < 0)
        {
            while ( distanceV > inch && opModeIsActive()) {
                getData();
                bot.driveTrain.spinDrive(speed, 0, 0);
                getTelem(); }
        }
        else

        {
            while ( distanceV > inch && opModeIsActive()) {
                getData();
                bot.driveTrain.spinDrive(speed, 0, 0);
                getTelem(); }
        }

        resetEncoders();
    }
}
