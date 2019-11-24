package org.westtorrancerobotics.lib.ftc;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.westtorrancerobotics.lib.hardware.drive.MecanumDrive;
import org.westtorrancerobotics.lib.spline.geom.Angle;

public class MecanumDriveImpl implements MecanumDrive {

    private final DcMotorEx leftFront;
    private final DcMotorEx leftBack;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightBack;

    public MecanumDriveImpl(DcMotorEx leftFront, DcMotorEx leftBack,
                            DcMotorEx rightFront, DcMotorEx rightBack) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
    }

    @Override
    public void setMotorPowers(double frontLeft, double backLeft, double frontRight, double backRight) {
        leftFront.setPower(frontLeft);
        leftBack.setPower(backLeft);
        rightFront.setPower(frontRight);
        rightBack.setPower(backRight);
    }

    @Override
    public double getWheelbaseWidth() {
        return 14.544475;
    }
}
