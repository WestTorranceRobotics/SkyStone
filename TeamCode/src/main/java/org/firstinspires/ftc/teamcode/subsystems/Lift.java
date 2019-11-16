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
    private final int LEVEL_HEIGHT = (int) (4 * TICKS_PER_INCH);
    private final int MAX_LEVEL = 4;
    //private ElapsedTime runtime = new ElapsedTime();
    //private double integral = 0;
    private double FEEDFORWARD = .33;

    private double P;
    private double I;
    private double D;
    private double F;

    private PidfController velocityControl;
    private DoubleUnaryOperator velocityProfile;
    private ElapsedTime velocityProfileTimer;
    private double velocityProfileDuration;

    private State state;

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

        velocityControl = new PidfController(() -> P, () -> I, () -> D, (pos) -> F);
        double cpsMax = SPEED * TICKS_PER_INCH;
        velocityControl.setOutputRange(-cpsMax, cpsMax);
        velocityProfileTimer = new ElapsedTime();

        hadBlock = false;

    public void idle() {
        if (ButtonAndEncoderData.getLatest().isPressed(bottomLimit)) {
            if (state != State.IDLING) {
                leftMotor.setPower(IDLE_POWER);
                rightMotor.setPower(IDLE_POWER);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                state = State.IDLING;
            }
        } else {
            if (state != State.OFF) {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                state = State.OFF;
            }
        }
    }

    public void zero() {
        encoderZero = ButtonAndEncoderData.getLatest().getCurrentPosition(leftMotor);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        state = State.OFF;
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
        encoderTarget = level * LEVEL_HEIGHT + encoderZero;
        makeVelocityFunction(encoderTarget);
        state = State.MOVING;
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
        double output = Double.NaN;
        if ((level == 0 && ButtonAndEncoderData.getLatest().isPressed(bottomLimit)) || velocityProfile == null) {
            if (state == State.OFF) {
                return;
            }
            if (ButtonAndEncoderData.getLatest().getCurrentVelocity(leftMotor)
                    > (SAFE_TO_OFF_VELOCITY / TICKS_PER_INCH)) {
                output = velocityControl.getOutput(
                        ButtonAndEncoderData.getLatest().getCurrentVelocity(leftMotor),
                        0
                );
            } else {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                state = State.OFF;
                return;
            }
        }
        if (Double.isNaN(output)) {
            output = velocityControl.getOutput(
                    ButtonAndEncoderData.getLatest().getCurrentVelocity(leftMotor),
                    velocityProfile.applyAsDouble(velocityProfileTimer.seconds())
            );
        }
        state = profileFinished() ? State.IDLING : State.MOVING;
        leftMotor.setPower(output);
        rightMotor.setPower(output);
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