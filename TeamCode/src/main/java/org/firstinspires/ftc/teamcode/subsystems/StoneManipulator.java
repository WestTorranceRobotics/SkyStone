package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Code for the following systems:
 * intake
 * extender
 * nub grabber
 *
 *
 *
 * FORWARD AND BACKWARD GRASPED/UNCLASPED POSITIONS MUST BE ADJUSTED FOR THE FORWARD AND BACKWARD GRAB ARMS
 **/
public class StoneManipulator {

    private Servo stoneGrabBig;
    private Servo stoneGrabLittle;
    private DcMotor rightIntake;
    private DcMotor leftIntake;
    private final double BIG_GRABBER_CLASPED_POSITION = .4;
    private final double BIG_GRABBER_UNCLASPED_POSITION = .9;
    private final double LITTLE_GRABBER_CLASPED_POSITION = .4;
    private final double LITTLE_GRABBER_UNCLASPED_POSITION = .9;
    private State currentState;
    private boolean clasped;

    private static StoneManipulator instance = null;

    public static synchronized StoneManipulator getInstance() {
        return instance != null ? instance : (instance = new StoneManipulator());
    }

    private StoneManipulator() {}

    /**
     * Initializes hardware.
     *
     * Clasped/unclasped positions set the maximum ranges for the nub grabber servers.
     * Robot moves on initialization.
     *
     **/
    public void init(HardwareMap hardwareMap) {
        rightIntake = hardwareMap.get(DcMotor.class, "intakeRight/odometerRightY");
        leftIntake = hardwareMap.get(DcMotor.class, "intakeLeft/odometerLeftY");
        rightIntake.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);

        stoneGrabBig = hardwareMap.get(Servo.class, "nubGrabBig");
        stoneGrabLittle = hardwareMap.get(Servo.class, "nubGrabLittle");
        setGrabbed(false);
    }

    /**
     * Sets the power of the intake
     *
     * if state is EXTENDED, returns EXTENDED, otherwise returns INTAKING unless power is 0
     *
     * */
    public State setIntake (double power) {
        if (currentState == State.EXTENDED) {
            return currentState;
        }
        leftIntake.setPower(power);
        rightIntake.setPower(power);
        currentState = (power != 0) ? State.INTAKING : State.RESTING;
        return currentState;
    }

    public boolean setGrabbed (boolean grabbed) {
        double positionArmBig = grabbed ? BIG_GRABBER_CLASPED_POSITION : BIG_GRABBER_UNCLASPED_POSITION;
        double positionArmLittle = grabbed ? LITTLE_GRABBER_CLASPED_POSITION : LITTLE_GRABBER_UNCLASPED_POSITION;
        clasped = grabbed;
        stoneGrabBig.setPosition(positionArmBig);
        stoneGrabLittle.setPosition(positionArmLittle);
        return clasped;
    }

    public State getState() {
        return currentState;
    }

    public boolean isGrabbed () {
        return clasped;
    }

    public enum State {
        RESTING,
        EXTENDED,
        INTAKING
    }
}
