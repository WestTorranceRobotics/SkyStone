package org.westtorrancerobotics.lib.hardware.drive;

public interface EncoderedMecanumDrive extends MecanumDrive, EncoderedDriveBase {
    long getRightFrontEncoder();
    long getRightBackEncoder();
    long getLeftFrontEncoder();
    long getLeftBackEncoder();
    double getFrontWheelsY();
    double getBackWheelsY();
}
