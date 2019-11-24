package org.westtorrancerobotics.lib.hardware.sensors;

import org.westtorrancerobotics.lib.spline.geom.Angle;

public interface TestableGyro {
    Angle getHeading();
    boolean isWorking();
}
