package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    private CRServo outtakeRight;
    private CRServo outtakeLeft;
    private boolean clasped;
    private Servo nubGrabber;
    private DcMotor rightIntake;
    private DcMotor leftIntake;
    private final double GRABBER_CLASPED_POSITION = 0;
    private final double GRABBER_UNCLASPED_POSITION = 0.48;
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
        nubGrabber = hardwareMap.get(Servo.class, "nubGrabber");
        outtakeLeft = hardwareMap.get(CRServo.class, "fourBarLeft");
        outtakeRight = hardwareMap.get(CRServo.class, "fourBarRight");

        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
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
     * @return
     */
    public State setFourBarSpeed (double speed) {
        outtakeLeft.setPower(speed);
        outtakeRight.setPower(speed);
        return currentState;
     } public boolean setGrabbed (boolean grabbed) {
        nubGrabber.setPosition(grabbed ? GRABBER_CLASPED_POSITION : GRABBER_UNCLASPED_POSITION);
        return grabbed;
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
