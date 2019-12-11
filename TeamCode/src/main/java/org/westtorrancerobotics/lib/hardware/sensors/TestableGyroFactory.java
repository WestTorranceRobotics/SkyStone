package org.westtorrancerobotics.lib.hardware.sensors;

import org.westtorrancerobotics.lib.spline.geom.Angle;

public class TestableGyroFactory {
    
    private TestableGyroFactory() {} // no constructor
    
    public static TestableGyro generate(TestableGyro... gyros) {
        return new TestableGyro() {
            @Override
            public Angle getHeading() {
                for (TestableGyro gyro : gyros) {
                    if (gyro.isWorking()) {
                        return gyro.getHeading();
                    }
                }
                return new Angle(0, 1);
            }

            @Override
            public boolean isWorking() {
                for (TestableGyro gyro : gyros) {
                    if (gyro.isWorking()) {
                        return true;
                    }
                }
                return false;
            }
        };
    }

    public static TestableGyro NULL = new TestableGyro() {
        @Override
        public Angle getHeading() {
            return new Angle(0, Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING);
        }

        @Override
        public boolean isWorking() {
            return false;
        }
    };
    
}
