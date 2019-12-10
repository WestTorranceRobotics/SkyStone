package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Mecanum Teleop", group="Linear Opmode")
//@Disabled
public class MecanumTeleop extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor left1;
    private DcMotor left2;
    private DcMotor right1;
    private DcMotor right2;

    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    private DcMotor liftRight;
    private DcMotor liftLeft;

    private Servo foundationHookRight;
    private Servo foundationHookLeft;

    private Servo grabRight;
    private Servo grabLeft;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        left1 = hardwareMap.dcMotor.get("leftFront");
        left2 = hardwareMap.dcMotor.get("leftBack");
        right1 = hardwareMap.dcMotor.get("rightFront");
        right2 = hardwareMap.dcMotor.get("rightBack");

        liftRight = hardwareMap.get(DcMotor.class, "liftRight/odometerX");
        liftRight.setDirection(DcMotor.Direction.FORWARD);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft/odometerLeftY");
        intakeRight = hardwareMap.dcMotor.get("intakeRight/odometerRightY");

        liftRight = hardwareMap.dcMotor.get("liftRight/odometerX");
        liftLeft = hardwareMap.dcMotor.get("liftLeft");

        grabLeft = hardwareMap.servo.get("nubGrabLittle");
        grabRight = hardwareMap.servo.get("nubGrabBig");

        left1.setDirection(DcMotorSimple.Direction.REVERSE);
        left2.setDirection(DcMotorSimple.Direction.REVERSE);

        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

            telemetry.update();

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

            if (gamepad2.right_bumper) {
                intakeSpeed = 1;
            }

            if (gamepad2.left_bumper) {
                intakeSpeed = -1;
            }
            intakeLeft.setPower(intakeSpeed);
            intakeRight.setPower(intakeSpeed);

            if (gamepad2.dpad_up) {
                liftLeft.setPower(.75);
                liftRight.setPower(.75);
            }
            else if (gamepad2.dpad_down) {
                liftRight.setPower(-.25);
                liftLeft.setPower(-.25);
            }
            else {
               liftRight.setPower(0);
               liftLeft.setPower(0);
            }

            if (!prevPos && gamepad2.x) {
                if (!pos) {
                    grabRight.setPosition(.753);
                    grabLeft.setPosition(.159);
                } else {
                    grabRight.setPosition(0.220);
                    grabLeft.setPosition(0.665);
                }
                pos = !pos;
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