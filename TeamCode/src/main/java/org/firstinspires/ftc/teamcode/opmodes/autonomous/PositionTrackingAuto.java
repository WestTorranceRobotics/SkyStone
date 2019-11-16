package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import static java.lang.Math.sqrt;

@Disabled
@Autonomous(name="Position Tracking", group="Linear Opmode")
public class PositionTrackingAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor left1;
    private DcMotor left2;
    private DcMotor right1;
    private DcMotor right2;

    private ColorSensor color;

    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private Servo outtakeLeft;
    private Servo outtakeRight;

    private DcMotor liftRight;
    private DcMotor liftLeft;

    private Servo grabRight;
    private Servo grabLeft;

    private Servo nubGrabRight;

    private Servo nubGrabLeft;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        left1 = hardwareMap.dcMotor.get("leftFront");
        left2 = hardwareMap.dcMotor.get("leftBack");
        right1 = hardwareMap.dcMotor.get("rightFront");
        right2 = hardwareMap.dcMotor.get("rightBack");

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft/odometerLeftY");
        intakeRight = hardwareMap.dcMotor.get("intakeRight/odometerRightY");

        liftRight = hardwareMap.dcMotor.get("liftRight/odometerX");
        liftLeft = hardwareMap.dcMotor.get("liftLeft");

        grabLeft = hardwareMap.servo.get("grabLeft");
        grabRight = hardwareMap.servo.get("grabRight");

        int leftOdo = intakeLeft.getCurrentPosition();
        int rightOdo = intakeRight.getCurrentPosition();
        int alignOdo = liftRight.getCurrentPosition();

        left1.setDirection(DcMotorSimple.Direction.REVERSE);
        left2.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        telemetry.addData("leftodo", leftOdo);
        telemetry.addData("rightodo", rightOdo);
        telemetry.addData("alignodo", alignOdo);
        telemetry.update();
    }



    int numOfTicks = 4096;
    double pi = 3.14159265358979323;
    int wheelDiameter = 2;
    double conversion = numOfTicks / (wheelDiameter * pi);

    void odomTutorial (int x, int y){

        int leftOdo = intakeLeft.getCurrentPosition();
        int rightOdo = intakeRight.getCurrentPosition();
        int alignOdo = liftRight.getCurrentPosition();

        int initialX = 0;
        int initialY = 0;
        int initialTheta = 0;

        double diagonalMovement = sqrt((x * x) + (y * y));
        double ticksMovement = diagonalMovement * conversion;

        int currentLeftPos = leftOdo;
        int currentRightPos = rightOdo;
        int currentAlignPos = alignOdo;

        double targetLeftPos = ticksMovement + currentLeftPos;
        double targetRightPos = ticksMovement + currentRightPos;

        //If x is less than 0;
        if (x < 0 && y > 0){
            if (leftOdo.getCurrentPosition() != targetLeftPos && rightOdo.getCurrentPosition() != targetRightPos) {
                left2.setPower(1);
                right1.setPower(1);
            }
        }
        else if (x < 0 && y < 0) {
            if (leftOdo.getCurrentPosition() != targetLeftPos && rightOdo.getCurrentPosition() != targetRightPos) {
                left1.setPower(-1);
                right2.setPower(-1);
            }
        }

        //If x is greater than 0

        else if (x > 0 && y < 0){
            if (leftOdo.getCurrentPosition() != targetLeftPos && rightOdo.getCurrentPosition() != targetRightPos) {
                left2.setPower(-1);
                right1.setPower(-1);
            }
        }

        else if (x > 0 && y > 0){
            if (leftOdo.getCurrentPosition() != targetLeftPos && rightOdo.getCurrentPosition() != targetRightPos) {
                left2.setPower(1);
                right1.setPower(1);
            }
        }

        //Everything else;

        else {
            left2.setPower(0);
            right1.setPower(0);
        }

//        if (leftOdo.getCurrentPosition() != targetLeftPos && rightOdo.getCurrentPosition() != targetRightPos) {
//            left1.setPower(-1);
//            right2.setPower(-1);
//        }
//        else {
//            left2.setPower(0);
//            right1.setPower(0);
//        }


    }

//    void moveToPosition(int x, int y, int angle, int turn) {
//        double diagonalMovement = sqrt((x * x) + (y * y));
//        double ticksMovement = diagonalMovement * conversion;
//
//        int currentLeftPos = leftOdo.getCurrentPosition();
//        int currentRightPos = rightOdo.getCurrentPosition();
//        int curentAlignPos = alignOdo.getCurrentPosition();
//
//        double targetLeftPos = ticksMovement + currentLeftPos;
//        double targetRightPos = ticksMovement + currentRightPos;
//
//
//
//        if (angle < 90 && angle > 0) {
//            while (currentLeftPos != leftOdo.getCurrentPosition()) {
//                left1.setPower(1);
//                right2.setPower(1);
//            }
//        }
//        if (angle < 180 && angle > 90) {
//            while (currentLeftPos != leftOdo.getCurrentPosition()) {
//                left2.setPower(1);
//                right1.setPower(1);
//            }
//        }
//        if (angle < 0 && angle > -90) {
//            while (currentLeftPos != leftOdo.getCurrentPosition()) {
//                left2.setPower(-1);
//                right1.setPower(-1);
//            }
//        }
//        if (angle < -90 && angle > -180) {
//            while (currentLeftPos != leftOdo.getCurrentPosition()) {
//                left1.setPower(-1);
//                right2.setPower(-1);
//            }
//        }
//    }
}
