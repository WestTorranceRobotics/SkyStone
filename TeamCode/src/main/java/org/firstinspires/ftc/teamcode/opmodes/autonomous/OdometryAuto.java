package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Base64;

@Disabled
@Autonomous(name="Odometry Auto", group="Linear Opmode")
public class OdometryAuto extends LinearOpMode {
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

        telemetry.addData("leftodo", left1.getCurrentPosition());
        telemetry.update();




        straightMovement(15, 1, 1, 1, 1);
        tracking.odomTutorial(1,1);
        sideToSide(5,1,-1,-1,1);


        //sideToSide(6, 1, -1, -1, 1);
    }

    int numOfTicks = 4096;
    double pi = 3.14159265358979323;
    int wheelDiameter = 2;
    double conversion = numOfTicks / (wheelDiameter * pi);

    void straightMovement(int inches, int L1, int L2, int R1, int R2) {

        int currentPosLeft =  intakeLeft.getCurrentPosition();
        // insert name of left odometry wheel
        int currentPosRight = intakeRight.getCurrentPosition();
        // insert name of the right odometry wheel

        double ticksMovement = conversion * inches;

        double targetPosLeft = currentPosLeft + ticksMovement;
        double targetPosRight = currentPosRight + ticksMovement;

        if (currentPosLeft != targetPosLeft && currentPosRight != targetPosRight) {
            left1.setPower(L1);
            left2.setPower(L2);
            right1.setPower(R1);
            right2.setPower(R2);
        } else {
            left1.setPower(0);
            left2.setPower(0);
            right1.setPower(0);
            right2.setPower(0);
        }
    }

    void sideToSide(int inches, int L1, int L2, int R1, int R2) {

        int currentPosLeft = intakeLeft.getCurrentPosition();
        // insert name of left odometry wheel
        int currentPosRight = intakeRight.getCurrentPosition();
        // insert name of the right odometry wheel
        int currentAlignment = liftRight.getCurrentPosition();

        double ticksMovement = conversion * inches;

        double targetAlignPos = currentAlignment + ticksMovement;

        if (currentAlignment != targetAlignPos) {
            left1.setPower(L1);
            left2.setPower(L2);
            right1.setPower(R1);
            right2.setPower(R2);
        }
        else {
            left1.setPower(0);
            left2.setPower(0);
            right1.setPower(0);
            right2.setPower(0);
        }
    }
}