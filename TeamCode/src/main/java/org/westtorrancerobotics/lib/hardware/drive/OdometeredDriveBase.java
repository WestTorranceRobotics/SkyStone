package org.westtorrancerobotics.lib.hardware.drive;

import org.westtorrancerobotics.lib.spline.geom.Location;

public interface OdometeredDriveBase {
    Location getLocation();
    // include closed loop control within this method
    void moveTowardLocation(Location target, double velocity);
}
