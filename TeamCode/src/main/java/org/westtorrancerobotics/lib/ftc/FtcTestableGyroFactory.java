package org.westtorrancerobotics.lib.ftc;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.westtorrancerobotics.lib.spline.geom.Angle;

public class FtcTestableGyroFactory {

    private FtcTestableGyroFactory() {} // no constructor

    static TestableGyro generate(BNO055IMU gyro) {
        return new TestableGyro() {
            @Override
            public Angle getHeading() {
                return new Angle(gyro.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX,
                        AngleUnit.DEGREES).firstAngle, Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING);
            }

            @Override
            public boolean isWorking() {
                return gyro.getSystemStatus() == (gyro.getParameters().mode.isFusionMode() ?
                        BNO055IMU.SystemStatus.RUNNING_FUSION : BNO055IMU.SystemStatus.RUNNING_NO_FUSION);
            }
        };
    }

    static TestableGyro generate(ModernRoboticsI2cGyro gyro) {
        return new TestableGyro() {
            @Override
            public Angle getHeading() {
                return new Angle(gyro.getIntegratedZValue(), Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING);
            }

            @Override
            public boolean isWorking() {
                return gyro.getZAxisScalingCoefficient() != 0;
            }
        };
    }

}
