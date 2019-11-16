package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber;

@Autonomous(name="NewTest", group="Linear Opmode")
@Disabled
public class NewTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    FoundationGrabber grab;
    Robot bot;
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

        bot = Robot.getInstance();

        bot.init(hardwareMap);

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

        double leftSight = grab.getDistance(FoundationGrabber.Hook.LEFT);
        double rightSight = grab.getDistance(FoundationGrabber.Hook.RIGHT);
        telemetry.addData("Distance: Left ", leftSight);
        telemetry.addData("Distance: Right", rightSight);

        waitForStart();

        runtime.reset();
        runtime.startTime();

        while (runtime.seconds() !=3) {
            left1.setPower(0.8);
            left2.setPower(0.8);
            right1.setPower(0.8);
            right2.setPower(0.8);
        }

        while(opModeIsActive()) {
            telemetry.addData("RED", bot.driveTrain.lineSpotter.red());
            telemetry.addData("BLUE", bot.driveTrain.lineSpotter.blue());
            telemetry.addData("GREEN", bot.driveTrain.lineSpotter.green());
            telemetry.update();
        }


        driveDistance(30,0,Direction.FORWARD);

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
    int NUM_OF_TICKS = 4096;
    double PI = 3.14159265358979323;
    int WHEEL_DIAMETER = 2;
    double conversion = NUM_OF_TICKS / (WHEEL_DIAMETER * PI);
    double TIME;
    double DISTANCE;
    double convert = TIME/DISTANCE;
    double power = 0.8;
    double DistanceX = 3;
    double POWEROVERDISTANCEX = power/DistanceX; //FORWARD MOVEMENT
    double DistanceY = 3;
    double POWEROVERDISTANCEY = power/DistanceY; //SIDE TO SIDE MOVEMENT



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
            if (FoundationGrabber.SeenObject.SKYSTONE == bot.foundationGrabber.getView(FoundationGrabber.Hook.RIGHT, right)) {
                bot.foundationGrabber.setGrabbed(FoundationGrabber.Hook.RIGHT, true);
                return true;
            }
            if (FoundationGrabber.SeenObject.SKYSTONE == bot.foundationGrabber.getView(FoundationGrabber.Hook.LEFT, left)) {
                bot.foundationGrabber.setGrabbed(FoundationGrabber.Hook.LEFT, true);
                return true;
            } else {
                bot.foundationGrabber.setGrabbed(FoundationGrabber.Hook.BOTH, false);
                return false;
            }
        }

        public void driveDistance(double x, double y, Direction direction){

            double distance = (long) Math.hypot(x, y);

            double time = distance * convert;

            double rate = distance/time;

            double distanceToPower;

            runtime.reset();
            runtime.startTime();

            if(direction == Direction.FORWARD){
                distanceToPower = POWEROVERDISTANCEX * rate;

                if (runtime.seconds() != time) {

                    left1.setPower(distanceToPower);
                    left2.setPower(distanceToPower);
                    right1.setPower(distanceToPower);
                    right2.setPower(distanceToPower);

                }
            }
            if(direction == Direction.BACKWARD){

                distanceToPower = POWEROVERDISTANCEX * rate;

                if (runtime.seconds() != time) {

                    left1.setPower(-distanceToPower);
                    left2.setPower(-distanceToPower);
                    right1.setPower(-distanceToPower);
                    right2.setPower(-distanceToPower);
                }
            }

            if(direction == Direction.LEFT){

                distanceToPower = POWEROVERDISTANCEY * distance;

                while (runtime.seconds() != time) {

                    left1.setPower(-distanceToPower);
                    left2.setPower(distanceToPower);
                    right1.setPower(distanceToPower);
                    right2.setPower(-distanceToPower);

                }
            }

            if(direction == Direction.RIGHT){

                distanceToPower = POWEROVERDISTANCEY * distance;

                while (runtime.seconds() != time){
                    left1.setPower(distanceToPower);
                    left2.setPower(-distanceToPower);
                    right1.setPower(-distanceToPower);
                    right2.setPower(distanceToPower);
                }

            }

        }

//        public void forwardMovement(double distance, boolean forward, double time) {
//
//        //Can be backwards too.
//
//        double position = 0;
//        double power = 0.8;
//        double DistanceX = 3;
//        double convert = power/DistanceX; //Equation for power is (Power * Time)/ distance
//
//        //Long sleepTime = (long) Math.hypot(x, y)
//
//            //Test the power to time to distance.. I.e Test at a certain level of power, how far and long it takes to get a spot. Remember!!!! STILL NEED TO TEST!!!!
//
//        double rate = distance/time;
//
//        double distanceToPower = rate * convert;
//
//        if (distanceToPower > 1) {
//            distanceToPower = 1;
//            time = convert * distance;
//        }
//
//        if (forward == false){
//            distanceToPower *= -1;
//        }
//
//        runtime.reset();
//
//        while (distance != position && runtime.seconds() <= time) {
//
//            left1.setPower(distanceToPower);
//            left2.setPower(distanceToPower);
//            right1.setPower(distanceToPower);
//            right2.setPower(distanceToPower);
//
//            telemetry.addData("Time", time);
//            telemetry.addData("Distance", distance);
//
//            telemetry.update();
//            sleep(1000);
//
//            position += rate;
//        }
//
//        }

        enum Direction{
            LEFT,
            RIGHT,
            FORWARD,
            BACKWARD
        }
    }

