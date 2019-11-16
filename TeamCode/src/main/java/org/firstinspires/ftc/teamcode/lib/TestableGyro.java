package org.firstinspires.ftc.teamcode.lib;

import org.westtorrancerobotics.lib.Angle;

public interface TestableGyro {
    Angle getHeading();
    boolean isWorking();
}
