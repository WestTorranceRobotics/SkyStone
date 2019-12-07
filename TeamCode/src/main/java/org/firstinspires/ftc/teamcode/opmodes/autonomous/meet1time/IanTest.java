package org.firstinspires.ftc.teamcode.opmodes.autonomous.meet1time;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="NewTest", group="Linear Opmode")
public class IanTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Robot bot;

    private DcMotor left1;
    private DcMotor left2;
    private DcMotor right1;
    private DcMotor right2;
    private RevColorSensorV3 leftEye;


//    private ColorSensor color;
//    // only for encoder use

    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    //    // only for motor outtake right motor
//    private Servo outtakeLeft;
//    // only for motor outtake left
//    private Servo outtakeRight;
//    // only for motor outtake right
//
    private DcMotor liftRight;
    private DcMotor liftLeft;

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        bot = Robot.getInstance();

        bot.init(hardwareMap);

        bot.driveTrain.init(hardwareMap);

        bot.foundationGrabber.init(hardwareMap);

        left1 = hardwareMap.dcMotor.get("leftFront");
        left2 = hardwareMap.dcMotor.get("leftBack");
        right1 = hardwareMap.dcMotor.get("rightFront");
        right2 = hardwareMap.dcMotor.get("rightBack");

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft/odometerLeftY");
        intakeRight = hardwareMap.dcMotor.get("intakeRight/odometerRightY");

//        outtakeLeft = hardwareMap.servo.get("outtakeLeft");
//        outtakeRight = hardwareMap.servo.get("outtakeRight");
//
        liftLeft = hardwareMap.dcMotor.get("liftLeft");
        liftRight = hardwareMap.dcMotor.get("liftRight/odometerX");

        left1.setDirection(DcMotorSimple.Direction.REVERSE);
        left2.setDirection(DcMotorSimple.Direction.REVERSE);

        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        intakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        runtime.reset();

        while (1 == 1) {
            telemetry.addData("leftOdo1", left1.getCurrentPosition());
            telemetry.addData("leftOdo2", left2.getCurrentPosition());
            telemetry.addData("rightOdo1", right1.getCurrentPosition());
            telemetry.addData("rightOdo2", right2.getCurrentPosition());
            telemetry.update();
        }


    }

    int NUM_OF_TICKS = 4096;
    double PI = 3.14159265358979323;
    int WHEEL_DIAMETER = 2;
    double conversion = NUM_OF_TICKS / (WHEEL_DIAMETER * PI);

    double TIMEX = 1000;
    double DISTANCEX = 36;

    double TIMEY = 0;
    double DISTANCEY = 0;

    double convertX = TIMEX / DISTANCEX;
    double convertY = TIMEY / DISTANCEY;

    double power = 0.8 * 1000; //power *milliseconds

    double DistanceX = 36;

    double POWEROVERDISTANCEX = power / DistanceX; //FORWARD MOVEMENT (in what units?) in

    double DistanceY = 25;
    double POWEROVERDISTANCEY = power / DistanceY; //SIDE TO SIDE MOVEMENT in

    void movement(Direction direction) {

        if (direction == Direction.LEFT) {
            left1.getCurrentPosition();
        }
    }

    enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }
}
