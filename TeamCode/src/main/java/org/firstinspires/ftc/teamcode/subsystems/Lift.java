package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.ButtonAndEncoderData;
public class Lift {
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private RevColorSensorV3 blockDetector;
    private final double MAX_NO_BLOCK_DIST_IN = 1;
    private boolean hadBlock;
    private RevTouchSensor bottomLimit;
    private long encoderZero;
    private long encoderTarget;
    private int level;
    private final int MAX_LEVEL = 4;
    //private ElapsedTime runtime = new ElapsedTime();
    //private double integral = 0;
    private double FEEDFORWARD = .33;

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
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
    }

    public void idle() {
    }

    public void zero() {
    }

    public void setLevel(int level) {
        if (level > MAX_LEVEL) {
            level = MAX_LEVEL;
        }
        if (level < 0) {
            level = 0;
        }
        if (level == this.level) {
            return;
        }
        this.level = level;
    }

    public int getLevel() {
        return level;
    }

    public void moveUp() {
        setLevel(level + 1);
    }

    public void moveDown() {
        setLevel(level - 1);
    }

    public void updatePosition() {
    }

    public boolean hasBlock() {
        return blockDetector.getDistance(DistanceUnit.INCH) < MAX_NO_BLOCK_DIST_IN;

    }

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