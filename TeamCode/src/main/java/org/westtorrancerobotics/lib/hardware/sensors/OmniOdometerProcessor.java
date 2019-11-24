package org.westtorrancerobotics.lib.hardware.sensors;

import org.westtorrancerobotics.lib.spline.geom.Angle;
import org.westtorrancerobotics.lib.spline.geom.Location;
import org.westtorrancerobotics.lib.util.MathUtils;

public class OmniOdometerProcessor {
    private Location myLocation;
    private final OmniOdometer odometer;
    
    private final double wheelAdir;
    private final double wheelBdir;
    private final double wheelCdir;
    
    private double wheelAlast;
    private double wheelBlast;
    private double wheelClast;

    public OmniOdometerProcessor(Location myLocation, OmniOdometer odometer) {
        this.myLocation = myLocation;
        this.odometer = odometer;
        wheelAdir = odometer.getRelativeLocationWheelA().direction
                .getValue(Angle.AngleUnit.RADIANS, Angle.AngleOrientation.COMPASS_HEADING);
        wheelBdir = odometer.getRelativeLocationWheelB().direction
                .getValue(Angle.AngleUnit.RADIANS, Angle.AngleOrientation.COMPASS_HEADING);
        wheelCdir = odometer.getRelativeLocationWheelC().direction
                .getValue(Angle.AngleUnit.RADIANS, Angle.AngleOrientation.COMPASS_HEADING);
        wheelAlast = odometer.getEncoderDistanceWheelA();
        wheelBlast = odometer.getEncoderDistanceWheelB();
        wheelClast = odometer.getEncoderDistanceWheelC();
    }

    public void update() {
        double dA = odometer.getEncoderDistanceWheelA() - wheelAlast;
        wheelAlast += dA;
        double dB = odometer.getEncoderDistanceWheelB() - wheelBlast;
        wheelBlast += dB;
        double dC = odometer.getEncoderDistanceWheelC() - wheelClast;
        wheelClast += dC;
        double[] solved = MathUtils.solveAugmentedMatrix(new double[][]{
                {
                        Math.cos(wheelAdir),
                        -Math.sin(wheelAdir),
                        -dA,
                        Math.cos(wheelAdir) * odometer.getRelativeLocationWheelA().x
                                - Math.sin(wheelAdir) * odometer.getRelativeLocationWheelA().y
                },
                {
                        Math.cos(wheelBdir),
                        -Math.sin(wheelBdir),
                        -dB,
                        Math.cos(wheelBdir) * odometer.getRelativeLocationWheelB().x
                                - Math.sin(wheelBdir) * odometer.getRelativeLocationWheelB().y
                },
                {
                        Math.cos(wheelCdir),
                        -Math.sin(wheelCdir),
                        -dC,
                        Math.cos(wheelCdir) * odometer.getRelativeLocationWheelC().x
                                - Math.sin(wheelCdir) * odometer.getRelativeLocationWheelC().y
                }
            }
        );
        double rotCenterRelX = solved[0];
        double rotCenterRelY = solved[1];
        double rotRadCw = 1 / solved[2];
        if (!Double.isFinite(rotCenterRelX) || !Double.isFinite(rotCenterRelY)
                || !Double.isFinite(rotRadCw) || MathUtils.isZero(rotRadCw)) {
            double[] xy = MathUtils.solveAugmentedMatrix(new double[][]{
                {Math.tan(wheelAdir), 1, dA / Math.cos(wheelAdir)},
                !MathUtils.isZero((wheelAdir - wheelBdir) % Math.PI) ?
                        new double[]{Math.tan(wheelBdir), 1, dB / Math.cos(wheelBdir)} :
                        new double[]{Math.tan(wheelCdir), 1, dC / Math.cos(wheelCdir)}
            });
            double dx = xy[0];
            double dy = xy[1];
            Angle thetaY = myLocation.direction;
            Angle thetaX = Angle.add(thetaY, Angle.EAST, Angle.AngleOrientation.COMPASS_HEADING);
            double fieldDx = dx * thetaX.getX() + dy * thetaY.getX();
            double fieldDy = dx * thetaX.getY() + dy * thetaY.getY();
            myLocation.translate(fieldDx, fieldDy);
            return;
        }
        double convT = myLocation.direction.getValue(Angle.AngleUnit.RADIANS, Angle.AngleOrientation.UNIT_CIRCLE);
        myLocation.direction = new Angle(
                myLocation.direction.getValue(Angle.AngleUnit.RADIANS, Angle.AngleOrientation.COMPASS_HEADING) + rotRadCw,
                Angle.AngleUnit.RADIANS,
                Angle.AngleOrientation.COMPASS_HEADING
        );
        double hw = myLocation.x;
        double kw = myLocation.y;
        double hr = hw + Math.sin(convT) * rotCenterRelX + Math.cos(convT) * rotCenterRelY;
        double kr = kw - Math.cos(convT) * rotCenterRelX + Math.sin(convT) * rotCenterRelY;
        double r = Math.hypot(hw-hr, kw-kr);
        double theta = -rotRadCw + Math.atan2(kw-kr, hw-hr);
        myLocation.setLocation(hr + r * Math.cos(theta), kr + r * Math.sin(theta));
    }
    
    public void setLocation(Location newPosition) {
        double dA = odometer.getEncoderDistanceWheelA() - wheelAlast;
        wheelAlast += dA;
        double dB = odometer.getEncoderDistanceWheelB() - wheelBlast;
        wheelBlast += dB;
        double dC = odometer.getEncoderDistanceWheelC() - wheelClast;
        wheelClast += dC;
        myLocation = newPosition;
    }
    
    public Location getLocation() {
        return myLocation;
    }
}
