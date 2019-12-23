package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber;
import org.westtorrancerobotics.lib.MecanumController;
import org.westtorrancerobotics.lib.MecanumDrive;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution;

@Autonomous(name="NewTest", group="Linear Opmode")
public class NewTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private Robot bot;
    // defining back right wheel
    private DcMotor left1;
    // defining back left wheel
    private DcMotor left2;
    // defining front right wheel
    private DcMotor right1;
    // defining back left wheel
    private DcMotor right2;
    private RevColorSensorV3 leftEye;

    private BNO055IMU backupGyro1;

    private double currentAngle;
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
    int i;
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
//    // only for the nub grabber lef

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        bot = Robot.getInstance();

        bot.init(hardwareMap);

        bot.driveTrain.init(hardwareMap);

        bot.foundationGrabber.init(hardwareMap);

        backupGyro1 = hardwareMap.get(BNO055IMU.class,"imu1");
        backupGyro1.initialize(new BNO055IMU.Parameters());

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
        //left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        telemetry.addData("hi",1);
        telemetry.update();

        runtime.reset();



        bot.foundationGrabber.setGrabbed(FoundationGrabber.Hook.RIGHT,true);
        bot.foundationGrabber.setGrabbed(FoundationGrabber.Hook.RIGHT,false);

        straightMovement(-22,-0.6);

            if (seeing(bot.foundationGrabber.getDistance(FoundationGrabber.Hook.LEFT), FoundationGrabber.Hook.LEFT)) {

                    sideToSide(5, 0.6);

                    sleep(1000);

                    straightMovement(-4,0.5);

                    bot.foundationGrabber.setGrabbed(FoundationGrabber.Hook.RIGHT, true);

                    sleep(2000);

                    i = 0;

            } else if (seeing(bot.foundationGrabber.getDistance(FoundationGrabber.Hook.RIGHT), FoundationGrabber.Hook.RIGHT)) {

                    straightMovement(-4,0.5);

                    sleep(1000);

                    bot.foundationGrabber.setGrabbed(FoundationGrabber.Hook.RIGHT, true);

                    sleep(2000);

                    i = 1;
            } else {
                    sideToSide(12, 0.6);

                    sleep(1000);

                    straightMovement(-4,0.6);

                    bot.foundationGrabber.setGrabbed(FoundationGrabber.Hook.RIGHT, true);

                    sleep(2000);

                    i =2;
            }


            straightMovement(8,0.5);

            if(i == 0){

                sideToSide(70,0.8);

                bot.foundationGrabber.setGrabbed(FoundationGrabber.Hook.BOTH,false);

                sleep(2000);

                sideToSide(-30,0.8);

            }
            else if(i ==1){
                sideToSide(64,0.8);

                bot.foundationGrabber.setGrabbed(FoundationGrabber.Hook.BOTH,false);

                sleep(2000);

                sideToSide(-30,0.8);
            }
            else if(i == 2){
                sideToSide(62,0.8);

                bot.foundationGrabber.setGrabbed(FoundationGrabber.Hook.BOTH,false);

                sleep(2000);

                sideToSide(-30,0.8);
            }



        //driveDistance(30,0,Direction.FORWARD);

//        while (opModeIsActive()) {
//            movement(30,0.8,0.8,0.8,0.8,5);
//
//            for (int i = 0; i<3; i++){
//                if(seeing(leftSight,rightSight)) {
//                    i = 3;
//                }
//                else{
//                    movement(5,0.8,0.8,0.8,0.8,1);
//                }
//                sleep(1000);
//            }
//            movement(5,-0.8,-0.8,-0.8,-0.8,2);
//
//            movement(60,0.8,0.8,0.8,0.8,10);
//
//
//            grab.setGrabbed(FoundationGrabber.Hook.BOTH, false);
//
//            movement(30,0.8,0.8,0.78,0.8,3);
//
//            break;
//
//        }
    }


    //int ENCODER_TICKS_PER_REVOLUTION = 1440; Of DCMOTOR. NOT MECANUM WHEEL!
    int NUM_OF_TICKS = 560;
    double PI = 3.14159265358979323;
    int WHEEL_DIAMETER = 4;
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


    void straightMovement(double inches, double POWER) {

        // insert name of the right odometry wheel
        double tick = conversion * inches;

        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(tick < 0) {
            while (left2.getCurrentPosition() > (int) tick) {
                telemetry.addData("left", left2.getCurrentPosition());
                telemetry.update();
                powerAll(POWER);
                forceAngle();
            }
        }
        else{
            while (left2.getCurrentPosition() < (int) tick) {
                telemetry.addData("left", left2.getCurrentPosition());
                telemetry.update();
                powerAll(POWER);
                forceAngle();
            }
        }

        powerAll(0);
        sleep(2000);

    }


    void sideToSide(double inches, double POWER) {

        double ticksMovement = conversion * inches;

        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(ticksMovement > 0){
            while (ticksMovement > left2.getCurrentPosition()) {
                left1.setPower(-POWER);
                left2.setPower(POWER);
                right1.setPower(POWER);
                right2.setPower(-POWER);
                forceAngle();
            }
        }else {
            while (ticksMovement < left2.getCurrentPosition()) {
                left1.setPower(POWER);
                left2.setPower(-POWER);
                right1.setPower(-POWER);
                right2.setPower(POWER);
                forceAngle();
            }
        }
        //SATHVIK HERE IS COMMENT BETA
        powerAll(0);
        sleep(2000);

    }
    boolean seeing (double distanceAway, FoundationGrabber.Hook side) {
        if (FoundationGrabber.SeenObject.SKYSTONE == bot.foundationGrabber.getView(side, distanceAway)) {
            return true;
        }
        else {
            return false;
        }
    }

    void powerAll(double power){
        left1.setPower(power);
        left2.setPower(power);
        right1.setPower(power);
        right2.setPower(power);
    }

    void allmotorsSet(DcMotor.RunMode MODE){
        left1.setMode(MODE);
        left2.setMode(MODE);
        right1.setMode(MODE);
        right2.setMode(MODE);
    }

    public void driveDistance(double distance, Direction direction, int time) {

        double distanceToPower;

        runtime.reset();

        if (direction == Direction.FORWARD) {
            distanceToPower = POWEROVERDISTANCEX * (distance/time);

            if (distanceToPower > 1){
                telemetry.addData("dumb boi", 1);
                distanceToPower = 0;
            }

            left1.setPower(distanceToPower);
            left2.setPower(distanceToPower);
            right1.setPower(distanceToPower);
            right2.setPower(distanceToPower);
            sleep(time);

            left1.setPower(0);
            left2.setPower(0);
            right1.setPower(0);
            right2.setPower(0);

            telemetry.addData("Power", distanceToPower);
            telemetry.update();

        }

        if (direction == Direction.BACKWARD) {
            distanceToPower = POWEROVERDISTANCEX * (distance/time);

            if (distanceToPower > 1){
                telemetry.addData("dumb boi", 1);
                distanceToPower = 0;
            }

            left1.setPower(-distanceToPower);
            left2.setPower(-distanceToPower);
            right1.setPower(-distanceToPower);
            right2.setPower(-distanceToPower);
            sleep(time);

            left1.setPower(0);
            left2.setPower(0);
            right1.setPower(0);
            right2.setPower(0);

            telemetry.addData("Direction: ", "Backward");
            telemetry.update();


        }

        if (direction == Direction.LEFT) {
            distanceToPower = POWEROVERDISTANCEY * (distance/time);

            if (distanceToPower > 1){
                telemetry.addData("dumb boi", 1);
                distanceToPower = 0;
            }

            left1.setPower(-distanceToPower);
            left2.setPower(distanceToPower);
            right1.setPower(distanceToPower);
            right2.setPower(-distanceToPower);
            sleep(time);

            telemetry.addData("Direction: ", "Left");
            telemetry.update();

        }

        if (direction == Direction.RIGHT) {

            distanceToPower = POWEROVERDISTANCEY * (distance/time);

            if (distanceToPower > 1){
                telemetry.addData("dumb boi", 1);
                distanceToPower = 0;
            }

            left1.setPower(distanceToPower);
            left2.setPower(-distanceToPower);
            right1.setPower(-distanceToPower);
            right2.setPower(distanceToPower);
            sleep(time);

            telemetry.addData("Direction: ", "Right");
            telemetry.update();
        }


    }

    private void forceAngle () {
        while ((currentAngle > 5 || currentAngle < -5) && opModeIsActive()) {
            if (!opModeIsActive()) {return;}
            currentAngle = backupGyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            if (currentAngle > 5 && opModeIsActive()) {
                left1.setPower(.25);
                left2.setPower(.25);
                right1.setPower(-.25);
                right2.setPower(-.25);
            } else if
            (currentAngle < -5 && opModeIsActive()) {
                left1.setPower(-.25);
                left2.setPower(-.25);
                right1.setPower(.25);
                right2.setPower(.25);
            } else {
                left1.setPower(0);
                left2.setPower(0);
                right1.setPower(0);
                right2.setPower(0);

            }
        }
    }


    public void forwardMovement(double distance) {

        //Can be backwards too.

        int ticks = (int) distance * 4096;

        left1.setTargetPosition(ticks);
        left2.setTargetPosition(ticks);
        right1.setTargetPosition(ticks);
        right2.setTargetPosition(ticks);

        left1.setPower(0.8);
        left2.setPower(0.8);
        right1.setPower(0.8);
        right2.setPower(0.8);

        left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    enum Direction {
        LEFT,
        RIGHT,
        FORWARD,
        BACKWARD
    }
}

