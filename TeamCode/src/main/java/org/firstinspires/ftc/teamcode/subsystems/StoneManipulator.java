package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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
    private static StoneManipulator instance = null;
    // limits maximum movement of extender
    //private CRServo outtakeRight;
    //private CRServo outtakeLeft;
    private boolean clasped;
    private Servo nubBig;
    private Servo nubLittle;
    private DcMotor rightIntake;
    private DcMotor leftIntake;
    private final double FORWARD_GRABBER_CLASPED_POSITION = .4;
    private final double FORWARD_GRABBER_UNCLASPED_POSITION = .9;
    private final double BACKWARD_GRABBER_CLASPED_POSITION = .4;
    private final double BACKWARD_GRABBER_UNCLASPED_POSITION = .9;
    private State currentState;
    public static synchronized StoneManipulator getInstance() {
        return instance != null ? instance : (instance = new StoneManipulator());
    }
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
        nubBig = hardwareMap.get(Servo.class, "nubGrabBig");
        nubLittle = hardwareMap.get (Servo.class, "nubGrabLittle");

        leftIntake.setDirection(DcMotor.Direction.REVERSE);
    }
  
    /**
     * Sets the power of the intake
     *
     * if state is EXTENDED, returns EXTENDED, otherwise returns INTAKING unless power is 0
     *
     * */
    public State setIntake (double power) {
        if (currentState == State.EXTENDED) {return currentState;}
        leftIntake.setPower(power);
        rightIntake.setPower(power);
        currentState = (power != 0) ? State.INTAKING : (power==0) ? State.RESTING: currentState;
        return currentState;
    }

    /**
     *
     * Sets if outtake is extended or not
     *
     *
     *
     * if intaking, return intaking, otherwise return extended lest power is 0
     * @param extended
     * @return
     */
    public State setExtended (boolean extended) {
       /* if (currentState == State.INTAKING) {return currentState;}
        double power = 1;
        power = (!extended) ? -power : power;
        outtakeRight.setPower(power);
        outtakeLeft.setPower(power);
        while (power != 0) {
            outtakeRight.setPower (extenderForwardLimit.isPressed() ? 0 : extenderReverseLimit.isPressed() ? 0 : power);
            outtakeLeft.setPower (extenderForwardLimit.isPressed() ? 0 : extenderReverseLimit.isPressed() ? 0 : power);
        }
        currentState = (power !=0) ? State.EXTENDED: (power == 0) ? State.RESTING : currentState;*/
        return currentState;
     } public boolean setGrabbed (boolean grabbed) {
       double positionArmBig = grabbed ? FORWARD_GRABBER_CLASPED_POSITION : FORWARD_GRABBER_UNCLASPED_POSITION;
       double positionArmLittle = grabbed ? BACKWARD_GRABBER_CLASPED_POSITION : BACKWARD_GRABBER_UNCLASPED_POSITION;
       clasped = (grabbed) ? true : false;
        nubBig.setPosition(positionArmBig);
        nubLittle.setPosition(positionArmLittle);
        return clasped;
    } public State getState() {
        return currentState;
    } public boolean isGrabbed () {
        return (clasped);
    }

    public enum State {
        RESTING,
        EXTENDED,
        INTAKING,
    }
}
