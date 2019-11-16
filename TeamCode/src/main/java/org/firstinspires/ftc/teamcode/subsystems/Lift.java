package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.ButtonAndEncoderData;
public class Lift {
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private RevColorSensorV3 blockDetector;
    private final double MAX_NO_BLOCK_DIST_IN = 1;
    private boolean hadBlock;
    private RevTouchSensor bottomLimit;
    private int level;
    private final int LEVEL_HEIGHT = 300; // ticks per lift level
    private final int MAX_LEVEL = 4;
    //private ElapsedTime runtime = new ElapsedTime();
    //private double integral = 0;
    private double FEEDFORWARD = .33;
  /*  private final double Kp = .2;
    private final double Ki = 1;
    private final double Kd = 1;
    private final double LEVEL_0 = 0;
    private final double LEVEL_1 = 125;
    private final double LEVEL_2 = 275;
    private final double LEVEL_3 = 450;
    private final double LEVEL_4 = 690;*/
    //private final double MAX_SPEED = 23.48;
    //private final double Ktemp = MAX_SPEED;
    //private double feed = 0;


    private static Lift instance = null;

    public static synchronized Lift getInstance() {
        return instance != null ? instance : (instance = new Lift());
    }

    private Lift() {}

    public void init(HardwareMap hardwareMap) {
        //integral = 0;
        //runtime.startTime();
        rightMotor = hardwareMap.get(DcMotorEx.class, "liftRight/odometerX");
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor = hardwareMap.get(DcMotorEx.class, "liftLeft");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        double aParam = 315;
        double bInvParam = 0.605;
        double cParam = 169.7;
        blockDetector = new RevColorSensorV3(
                hardwareMap.get(RevColorSensorV3.class, "liftBlockColor").getDeviceClient()
        ) {
            @Override
            protected double inFromOptical(int rawOptical) {
                return Math.pow((rawOptical - cParam) / aParam, -bInvParam);
            }
        };
        bottomLimit = hardwareMap.get(RevTouchSensor.class, "liftLimitSwitch");
        level = 0;

        hadBlock = false;
        wasIdling = false;
        wasStill = true;
    }

    private boolean wasIdling;
    private boolean wasStill;

    public void idle() {
        if (ButtonAndEncoderData.getLatest().isPressed(bottomLimit)) {
            if (!wasIdling) {
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftMotor.setPower(0.1);
                rightMotor.setPower(0.1);
                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                wasIdling = true;
            }
        } else {
            if (!wasStill) {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                wasStill = true;
            }
        }
    }

    public void zero() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setLevel(int level) {
        this.level = level;
        if (level > MAX_LEVEL) {
            this.level = MAX_LEVEL;
        }
        if (level < 0) {
            this.level = 0;
        }
    }

    public int getLevel() {
        return level;
    }

    public void moveUp() {
        level++;
        if (level > MAX_LEVEL) {
            level = MAX_LEVEL;
        }
    }

    public void moveDown() {
        level--;
        if (level < 0) {
            level = 0;
        }
    }

    public void updatePosition() {
        if (level != 0) {
            wasStill = false;
        }
        wasIdling = false;
        leftMotor.setTargetPosition(level * LEVEL_HEIGHT);
    }

    public void updateRightMotor() {
        if (level != 0) {
            rightMotor.setPower(Robot.getInstance().controlHub.getMotorVoltagePercent(leftMotor));
            wasStill = false;
        } else if (!wasStill) {
            rightMotor.setPower(0);
            wasStill = true;
        }
    }

    public boolean hasBlock() {
        return blockDetector.getDistance(DistanceUnit.INCH) < MAX_NO_BLOCK_DIST_IN;

    }
/*
    private double pidPos(double currentPos, double targetPos) {
        //double error = targetPos - currentPos;
        // if ((error < .1 && error != 0) || (error > .1 && error!= 0)) {error = 0;}
        //double proportional = error * Kp;
        //double targetVel = 1 * proportional;
        return FEEDFORWARD;
    }*/


    //  /* private double pidVel(double initialTime, double targetVel) {
    //    double currentVel = (leftMotor.getVelocity());
    //   double error = targetVel - currentVel;
    // if (targetVel == 0) {error =0;}
    //if ((error < .1 && error != 0) || (error > .1 && error!= 0)) {error = 0;}
    // double proportional = error * Kp;
    //    integral += error * (runtime.time() - initialTime) ;
    // integral = ( error == 0 ) ? 0 : integral;
    // double derivative = 0;//error / (runtime.time() - initialTime);
    //  double feedforward = FEEDFORWARD;
    // return ((feedforward + proportional > 1) ? 1 : feedforward + proportional);*/
    //   }

   /* public void pidProcess(int level) {
        //double initialTime = runtime.time();
        //  double targetPosition = (level == 0) ? LEVEL_0 :(level == 1) ? LEVEL_1 : (level == 2) ? LEVEL_2 : (level == 3) ? LEVEL_3 : (level == 4) ? LEVEL_4 : 0 ;
        //double currentPosition = leftMotor.getCurrentPosition();
        // replace later with limit switch code when switch is ready for use
        //if (currentPosition <15) {leftMotor.resetDeviceConfigurationForOpMode();}
        //double targetVel = pidPos(currentPosition, targetPosition);
        //double power = (pidPos (currentPosition, targetPosition));
        leftMotor.setPower(FEEDFORWARD);
        rightMotor.setPower(FEEDFORWARD);
    }*/

    public void liftMove(double input) {
        if (input < -.2)  {
            leftMotor.setPower(.75);
            rightMotor.setPower(.75);
        }
        else if (input > .15)  {
            leftMotor.setPower(-.05);
            rightMotor.setPower(-.05);
        } else if (StoneManipulator.getInstance().isGrabbed()) {
            rightMotor.setPower(0);
            leftMotor.setPower(0);}
        else{
            leftMotor.setPower(FEEDFORWARD);
            rightMotor.setPower(FEEDFORWARD);
        }
    }
    }



