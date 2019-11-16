package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber;

import java.util.Base64;

@Disabled
@Autonomous(name = "Test", group = "none")
public class Test extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private PositionTrackingAuto tracking = new PositionTrackingAuto();

    FoundationGrabber grab;
    DriveTrain train;
    // defining back right wheel
    private DcMotor left1;
    // defining back left wheel
    private DcMotor left2;
    // defining front right wheel
    private DcMotor right1;
    // defining back left wheel
    private DcMotor right2;
    // color sensor

//    private ColorSensor color;
//    // only for encoder use

    private DcMotor intakeLeft;
    // only for motor intake left motor
    private DcMotor intakeRight;

    //    // only for motor outtake right motor
//    private Servo outtakeLeft;
//    // only for motor outtake left
//    private Servo outtakeRight;
//    // only for motor outtake right
//
    private DcMotor liftRight;
    // only for elevator right motor
    private DcMotor liftLeft;
    int i = 0;
    // only for elevator left motor
//
//    private Servo grabRight;
//    // only for grabber right
//    private Servo grabLeft;
//    // only for grabber left
//
//    private Servo nubGrabRight;
//    // only for the nub grabber right
//    private Servo nubGrabLeft;
//    // only for the nub grabber left


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        left1 = hardwareMap.dcMotor.get("leftFront");
        left2 = hardwareMap.dcMotor.get("leftBack");
        right1 = hardwareMap.dcMotor.get("rightFront");
        right2 = hardwareMap.dcMotor.get("rightBack");

        grab.init(hardwareMap);


        intakeLeft = hardwareMap.dcMotor.get("intakeLeft/odometerLeftY");
        intakeRight = hardwareMap.dcMotor.get("intakeRight/odometerRightY");

//        outtakeLeft = hardwareMap.servo.get("outtakeLeft");
//        outtakeRight = hardwareMap.servo.get("outtakeRight");
//
        liftLeft = hardwareMap.dcMotor.get("liftLeft");
        liftRight = hardwareMap.dcMotor.get("liftRight/odometerX");
//
//        grabRight = hardwareMap.servo.get("grabRight");
//        grabLeft = hardwareMap.servo.get("grabLeft");
//
//        nubGrabLeft = hardwareMap.servo.get("nubGrabLeft");
//        nubGrabRight = hardwareMap.servo.get("nubGrabright");


        left1.setDirection(DcMotorSimple.Direction.REVERSE);
        left2.setDirection(DcMotorSimple.Direction.REVERSE);

        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double leftSight = grab.getDistance(FoundationGrabber.Hook.LEFT);
        double rightSight = grab.getDistance(FoundationGrabber.Hook.RIGHT);
        telemetry.addData("Distance: Left ", leftSight);
        telemetry.addData("Distance: Right", rightSight);



        waitForStart();

        telemetry.addData("leftodo", left1.getCurrentPosition());
        telemetry.update();


        straightMovement(30,0.5,0.5,0.5,0.5);


        for (int i = 0; i < 3; i++) {

            if (seeing(leftSight, rightSight) == true) {
                i = 3;
            }
            else{
                sideToSide(5,0.5,-0.5,-0.5,0.5);

            }

            sleep(2000);
        }

        straightMovement(15,-0.5,-0.5,-0.5,-0.5);

        sideToSide(60-(5*i), -0.8,0.8,-0.8,0.8);

        grab.setGrabbed(FoundationGrabber.Hook.BOTH, false);

        sideToSide(30, 0.8,-0.8,0.8,-0.8);

        //sideToSide(6, 1, -1, -1, 1);
    }



    int NUM_OF_TICKS = 4096;
    double PI = 3.14159265358979323;
    int WHEEL_DIAMETER = 2;
    double conversion = NUM_OF_TICKS / (WHEEL_DIAMETER * PI);

    void straightMovement(int inches, double L1, double L2, double R1, double R2) {

        int currentPosLeft =  intakeLeft.getCurrentPosition();
        // insert name of left odometry wheel
        int currentPosRight = intakeRight.getCurrentPosition();

        // insert name of the right odometry wheel

        double ticksMovement = conversion * inches;

        double targetPosLeft = currentPosLeft + ticksMovement;
        double targetPosRight = currentPosRight + ticksMovement;

        left1.setTargetPosition((int) targetPosLeft);
        left2.setTargetPosition((int) targetPosLeft);
        right1.setTargetPosition((int) targetPosRight);
        right2.setTargetPosition((int) targetPosRight);

        left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left1.setPower(L1);
        left2.setPower(L2);
        right1.setPower(R1);
        right2.setPower(R2);



    }

    void sideToSide(int inches, double L1, double L2, double R1, double R2) {

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
        } else {
            left1.setPower(0);
            left2.setPower(0);
            right1.setPower(0);
            right2.setPower(0);
        }
    }
        boolean seeing (double left, double right) {
            if (FoundationGrabber.SeenObject.SKYSTONE == grab.getView(FoundationGrabber.Hook.RIGHT, right)) {
                grab.setGrabbed(FoundationGrabber.Hook.RIGHT, true);
                return true;
            }
            if (FoundationGrabber.SeenObject.SKYSTONE == grab.getView(FoundationGrabber.Hook.LEFT, left)) {
                grab.setGrabbed(FoundationGrabber.Hook.LEFT, true);
                return true;
            } else {
                grab.setGrabbed(FoundationGrabber.Hook.BOTH, false);
                return false;
            }
        }
    }

