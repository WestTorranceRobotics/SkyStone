package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.StoneManipulator;

//import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="Mecanum Teleop", group="Linear Opmode")
//@Disabled
public class MecanumTeleop extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor left1;
    private DcMotor left2;
    private DcMotor right1;
    private DcMotor right2;

    private boolean isGrabbed;

    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private CRServo outtakeLeft;
    private CRServo outtakeRight;

    private Servo nubBig;
    private Servo nubLittle;

    private Servo grabRight;
    private Servo grabLeft;

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        left1 = hardwareMap.dcMotor.get("leftFront");
        left2 = hardwareMap.dcMotor.get("leftBack");
        right1 = hardwareMap.dcMotor.get("rightFront");
        right2 = hardwareMap.dcMotor.get("rightBack");

        rightMotor = hardwareMap.get(DcMotorEx.class, "liftRight/odometerX");
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor = hardwareMap.get(DcMotorEx.class, "liftLeft");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft/odometerLeftY");
        intakeRight = hardwareMap.dcMotor.get("intakeRight/odometerRightY");

        grabLeft = hardwareMap.servo.get("foundationHookLeft");
        grabRight = hardwareMap.servo.get("foundationHookRight");

        nubBig = hardwareMap.get(Servo.class, "nubGrabBig");
        nubLittle = hardwareMap.get (Servo.class, "nubGrabLittle");

        outtakeLeft = hardwareMap.crservo.get("outtakeLeft");
        outtakeRight = hardwareMap.crservo.get("outtakeRight");



        left1.setDirection(DcMotorSimple.Direction.REVERSE);
        left2.setDirection(DcMotorSimple.Direction.REVERSE);

        right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("right", grabRight.getPosition());
        telemetry.addData("left", grabRight.getPosition());
        telemetry.update();

        grabRight.setPosition(0.220);
        grabLeft.setPosition(0.665);

        boolean prevPos = false;
        boolean pos = false;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            float vertical = -(gamepad1.left_stick_y);
            float horizontal = gamepad1.left_stick_x;
            float pivot = gamepad1.right_stick_x / 2;

            setRight1Power(-pivot + (vertical - horizontal));
            setRight2Power(-pivot + (vertical + horizontal));
            setLeft1Power(pivot + (vertical + horizontal));
            setLeft2Power(pivot + (vertical - horizontal));

            left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            int intakeSpeed = 0;
            double outtakeSpeed = 0;

            if (gamepad2.right_bumper) {
                intakeSpeed = 1;
            }
            if (gamepad2.dpad_up) {leftMotor.setPower(.75); rightMotor.setPower(.75);} else if (gamepad2.dpad_down) {leftMotor.setPower(-.05); rightMotor.setPower(-.05);} else if (!isGrabbed) {leftMotor.setPower(0); rightMotor.setPower(0);} else {leftMotor.setPower(.33); rightMotor.setPower(.33);}

            if (gamepad2.dpad_right) {
                outtakeSpeed = -.75;
            } else if (gamepad2.dpad_left) { outtakeSpeed = .75;} else { outtakeSpeed = 0;}
            intakeLeft.setPower(intakeSpeed);
            intakeRight.setPower(intakeSpeed);
            outtakeRight.setPower(outtakeSpeed);
            outtakeLeft.setPower(outtakeSpeed);

            if (!prevPos && gamepad2.x) {
                if (!pos) {
                    grabRight.setPosition(0.61);
                    grabLeft.setPosition(0.31);
                    pos = !pos;
                } else {
                    grabRight.setPosition(0.076);
                    grabLeft.setPosition(0.847);
                    pos = !pos;
                }
                prevPos = true;
            } else if (!gamepad2.x) {
                prevPos = false;
            }
        }
        if (gamepad2.a) {nubBig.setPosition(.9); nubLittle.setPosition(.9); isGrabbed = true;} else if (gamepad2.b) {nubBig.setPosition(.4); nubLittle.setPosition(.4); isGrabbed = false;}
    }

    void setLeft2Power(double n){
        left2.setPower(n);
    }
    void setLeft1Power(double n){
        left1.setPower(n);
    }
    void setRight1Power(double n){
        right1.setPower(n);
    }
    void setRight2Power(double n){
        right2.setPower(n);
    }
}