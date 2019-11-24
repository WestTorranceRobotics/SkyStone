package org.westtorrancerobotics.lib.hardware.sensors;

import org.westtorrancerobotics.lib.spline.geom.Location;

public interface OmniOdometer {
    Location getRelativeLocationWheelA();
    Location getRelativeLocationWheelB();
    Location getRelativeLocationWheelC();
    double getEncoderDistanceWheelA();
    double getEncoderDistanceWheelB();
    double getEncoderDistanceWheelC();
}
