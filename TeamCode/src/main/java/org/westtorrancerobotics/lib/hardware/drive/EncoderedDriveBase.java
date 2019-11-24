package org.westtorrancerobotics.lib.hardware.drive;

/**
 * Specifies a drive base with encoders on its powering wheels. Super interface for
 * all drive base controllers that request encoder values. Gives constants for conversion
 * between encoder ticks and distance traveled.
 * 
 * @since 1.1
 */
public interface EncoderedDriveBase {
    
    /**
     * Specifies the diameter of the drive base wheels. For tank treads, this is the
     * amount of distance across one of the wheels moving the belt, equal to the quotient
     * of the distance of tread moved in one motor revolution and pi. Supply this
     * number in your standard linear distance unit (e.g, inches, meters). This will
     * be a constant throughout the program. The diameter of the wheel is used in
     * conversion of encoder ticks to distance traveled.
     * 
     * @return the wheel diameter of the drive base wheels
     * @since 1.1
     */
    public double getWheelDiameter();
    
    /**
     * Specifies the number of encoder ticks in one revolution of a side of the drive
     * base. A revolution is defined as 360 degrees (or an equivalent rotation) on
     * a drive shaft of the {@code DriveBase} This is a unitless quantity. This will
     * be a constant throughout the program. The encoder ticks per revolution are
     * used in conversion of encoder ticks to distance traveled.
     * 
     * @return the number of encoder ticks in one revolution of a drive shaft
     * @since 1.1
     */
    public double getEncoderTicksPerRevolution();
    
}
