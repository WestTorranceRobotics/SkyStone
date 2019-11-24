package org.westtorrancerobotics.lib.hardware.drive;

/**
 * A robot drive base with two parallel sides. The robot may have any number of wheels
 * and motors on each side, but it is assumed that both the wheels and motors on
 * each side are fixed to the same velocity for every input as their peers, whether
 * mechanically or by underlying implementation code. Although some of the wheels
 * may be omnidirectional, it is assumed that at least on wheel on each side will
 * have substantial traction, and that there will not be significant drive train
 * slippage during driving. It is also assumed that no Mecanum wheels have been used.
 * A front, back, left, and right should be assigned for use of this class.
 * 
 * @see org.westtorrancerobotics.lib.hardware
 * @since 1.0
 */
public interface TankDrive extends EncoderedDriveBase {
    
    /**
     * Set the driving powers for this tank drive. The left of the two parallel sides
     * of the drive base, when atop the base facing its front, and the right side
     * in the same perspective are controlled by this method. Power should scale
     * linearly as specified by this method, meaning that outbound stall torque, in
     * Newton-meters, shall by directly proportional to percent power. Inputs out
     * of the domain [-1, 1] shall not give more power than an input of 1 or -1.
     * A negative power shall specify opposite direction and equal torque to the
     * positive power of the same absolute value.
     * <p>
     * A positive power for both inputs should cause the robot to drive in the
     * direction of its front, and two negative inputs should cause the robot to
     * drive toward its back. Equal motor powers should not cause a spinning motion,
     * as the underlying implementation shall internally inverse the right side motors.
     * 
     * @param left percent power of the left side of the drive base
     * @param right percent power of the right side of the drive base
     * @see TankDrive
     * @since 1.0
     */
    public void setLeftRightPower(double left, double right);
    
    /**
     * Recalls the last power set to the left side of the drive train. If this
     * {@code TankDrive} is the only active controller of the motors of the drive
     * base, the number returned should be the most recent first input to
     * {@link #setLeftRightPower}, otherwise the method
     * should return the most recent percentage power set by the active controller.
     * 
     * @return the last power set to the left side of the drive train
     * @see #setLeftRightPower(double left, double right)
     * @since 1.0
     */
    public double getLeftPower();
    
    /**
     * Recalls the last power set to the right side of the drive train. If this
     * {@code TankDrive} is the only active controller of the motors of the drive
     * base, the number returned should be the most recent second input to
     * {@link #setLeftRightPower}, otherwise the method
     * should return the most recent percentage power set by the active controller.
     * 
     * @return the last power set to the right side of the drive train
     * @see #setLeftRightPower(double left, double right)
     * @since 1.0
     */
    public double getRightPower();
    
    /**
     * Gives the integer reading of the encoder sensor attached to the left side
     * of the drive base. The number returned by this method should be read directly
     * from hardware, without conversion to a standard distance unit, like inches.
     * Instead, the {@link getWheelDiameter} and {@link getEncoderTicksPerRevolution}
     * methods will indicate this conversion factor. The positive direction of the
     * encoder should be the same as the direction the motor turns under no torque
     * with positive power.
     * 
     * @return the reading of the left side encoder
     * @since 1.0
     */
    public long getLeftEncoder();
    
    /**
     * Gives the integer reading of the encoder sensor attached to the right side
     * of the drive base. The number returned by this method should be read directly
     * from hardware, without conversion to a standard distance unit, like inches.
     * Instead, the {@link getWheelDiameter} and {@link getEncoderTicksPerRevolution}
     * methods will indicate this conversion factor. The positive direction of the
     * encoder should be the same as the direction the motor turns under no torque
     * with positive power.
     * 
     * @return the reading of the right side encoder
     * @since 1.0
     */
    public long getRightEncoder();
    
    /**
     * Specifies the width of the drive base. The width is measured from the center
     * of the left wheels to the center of the right wheels. Supply this number in
     * your standard linear distance unit (e.g, inches, meters). This will be a constant
     * throughout the program.
     * <p>
     * This number is used to determine how sharply a power differential will turn
     * the drive base, which allows calculation of a power differential when a desired
     * turn rate is known.
     * 
     * @return the drive base width in distance units
     * @since 1.0
     */
    public double getWheelbaseWidth();
    
}
