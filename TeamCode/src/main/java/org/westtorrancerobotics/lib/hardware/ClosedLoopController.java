package org.westtorrancerobotics.lib.hardware;

/**
 * A controller that uses input to move a piece of hardware to a specified position.
 * Assumed to be constantly running in a background loop. The target and current position
 * are in the same units. Target is the position the controller is trying to bring
 * to hardware to and keep it at, while the current position is the reading of the
 * sensor attached to the component of the robot being driven by the hardware. PID
 * is the most common algorithm used to achieve this purpose, but PIDF, PIDVA, motion
 * profiles, and other algorithms can all be valid.
 * <p>
 * Whatever code uses a closed loop controller should input to it the target position
 * and the current position of the mechanism controlled. The {@link #getOutput}
 * method should be also be called by code using controllers, and its output should
 * be used in a way that affects the source used to determine the parameter of
 * {@link #setCurrentPosition}. The only function a closed loop controller
 * shall perform is the calculation of outputs based on inputs, it should neither
 * read inputs nor use outputs internally.
 * 
 * @see org.westtorrancerobotics.lib.hardware
 * @since 1.0
 */
public interface ClosedLoopController {
    
    double getOutput(double position, double target);
    
    /**
     * Assigns the controller a target position to reach. The general
     * contract of this method is that the controller will, as much as it is able,
     * choose an output that will drive it to and hold this target position. If
     * {@link #setCurrentPosition} is not called repeatedly while tracking a target,
     * the controller will have no ability to detect whether or not the position
     * is reached.
     * 
     * @param target the position for the controller to reach
     * @see #setCurrentPosition(double position)
     * @see #getOutput()
     * @since 1.0
     */
//    public void setTarget(double target);
    
    /**
     * Tells the controller where its mechanism is currently located. The controller
     * shall use this position as feedback to determine what output to return, given
     * that its goal is to adjust the output to make this method return the current
     * target as specified be {@link #setTarget}, assuming that the target
     * position will not change.
     * 
     * @param position the current location of the mechanism
     * @see #setTarget(double target)
     * @see #getOutput()
     * @since 1.0
     */
//    public void setCurrentPosition(double position);
    
    /**
     * Gives using code an output value with which to drive the mechanism. The same
     * system of code that informs the controller of current and target positions
     * should use the value returned by this method to tell the mechanism how it should
     * drive itself. For consistent and meaningful outputs to be read, it is necessary
     * that both the current and target positions are being actively updated.
     * 
     * @return a specification for the driving of this controller's mechanism
     * @see #setCurrentPosition(double position)
     * @see #setTarget(double target)
     * @since 1.0
     */
//    public double getOutput();
}
