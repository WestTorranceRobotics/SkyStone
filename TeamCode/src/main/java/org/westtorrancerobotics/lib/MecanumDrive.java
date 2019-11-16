package org.westtorrancerobotics.lib;

public interface MecanumDrive {
    void setMotorPowers(double frontLeft, double backLeft, double frontRight, double backRight);
    double getWheelbaseWidth();
    
    enum TranslationMethod {
        MAX_SPEED,
        CONSTANT_SPEED,
        CONSTANT_POWER
    }
    
    enum TranslTurnMethod {
        CONSTANT_TRANSLATION_SPEED, // specified move pwr, specified turn power *= remaining power
        EQUAL_POWERS, // 0.5 average move pwr, 0.5 average turn power
        EQUAL_SPEED_RATIOS, // which is max speed * min(specified powers)
        CONSTANT_BOTH_SPEED // supplied speed and turn parameters must sum to one
    }
}
