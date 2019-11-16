package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private ColorSensor color;

    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private CRServo outtakeLeft;
    private CRServo outtakeRight;
    private DcMotor liftLeft;
    private DcMotor liftRight;


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

        grabLeft = hardwareMap.servo.get("foundationHookLeft");
        grabRight = hardwareMap.servo.get("foundationHookRight");

        outtakeLeft = hardwareMap.crservo.get("outtakeLeft");
        outtakeRight = hardwareMap.crservo.get("outtakeRight");

        int leftOdo = intakeLeft.getCurrentPosition();
        int rightOdo = intakeRight.getCurrentPosition();
        int alignOdo = liftRight.getCurrentPosition();

        left1.setDirection(DcMotorSimple.Direction.REVERSE);
        left2.setDirection(DcMotorSimple.Direction.REVERSE);
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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
            float pivot = gamepad1.right_stick_x;

            setRight1Power(-pivot + (vertical - horizontal));
            setRight2Power(-pivot + (vertical + horizontal));
            setLeft1Power(pivot + (vertical + horizontal));
            setLeft2Power(pivot + (vertical - horizontal));

            left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addData("leftOdo", leftOdo);
            telemetry.addData("rightOdo", rightOdo);
            telemetry.addData("alignOdo", alignOdo);

            int intakeSpeed = 0;
            double outtakeSpeed = 0;

            if (gamepad2.right_bumper) {
                intakeSpeed = 1;
                outtakeSpeed = .75;
            }

            if (gamepad2.left_bumper) {
                intakeSpeed = -1;
                outtakeSpeed = -0.75;
            }
            intakeLeft.setPower(intakeSpeed);
            intakeRight.setPower(intakeSpeed);
            outtakeRight.setPower(outtakeSpeed);
            outtakeLeft.setPower(outtakeSpeed);

//            telemetry.addData("outtake speed R", outtakeRight.getPower());
//            telemetry.addData("outtake speed L", outtakeLeft.getPower());
//            telemetry.update();

           if (gamepad2.dpad_up) {
                liftLeft.setPower(.75);
                liftRight.setPower(.75);
           }
           else if (gamepad2.dpad_down) {
                liftRight.setPower(-.25);
                liftLeft.setPower(-.25);
           }
           else {
               liftRight.setPower(-.33);
               liftLeft.setPower(.33);
           }



//            double liftSpeed = .2;
//
//            if (gamepad2.dpad_up) {
//                liftSpeed = -.75;
//            }
//            if (gamepad2.dpad_down) {
//                liftSpeed =
//            }

//            telemetry.addData("right", grabRight.getPosition());
//            telemetry.addData("left", grabRight.getPosition());
//            telemetry.update();

            if (!prevPos && gamepad2.x) {
                if (!pos) {
                    grabRight.setPosition(.753);
                    grabLeft.setPosition(.159);
                    pos = !pos;
                } else {
                    grabRight.setPosition(0.220);
                    grabLeft.setPosition(0.665);
                    pos = !pos;
                }
                prevPos = true;
            } else if (!gamepad2.x) {
                prevPos = false;
            }
        }
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