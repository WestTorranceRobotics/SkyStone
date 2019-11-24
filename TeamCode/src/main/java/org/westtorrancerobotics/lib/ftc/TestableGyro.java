package org.westtorrancerobotics.lib.ftc;

import org.westtorrancerobotics.lib.spline.geom.Angle;

public interface TestableGyro {
    Angle getHeading();
    boolean isWorking();
}
