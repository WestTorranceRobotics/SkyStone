package org.westtorrancerobotics.lib.hardware.drive;

import org.westtorrancerobotics.lib.spline.geom.Angle;
import org.westtorrancerobotics.lib.spline.geom.Location;
import org.westtorrancerobotics.lib.util.MathUtils;

public class TankController implements OdometeredDriveBase {
    
    private final TankDrive drive;
    private double lastLeft;
    private double lastRight;
    
    private Location myLocation;

    public TankController(TankDrive drive) {
        this.drive = drive;
        lastLeft  = ticksToInches(drive.getLeftEncoder());
        lastRight = ticksToInches(drive.getRightEncoder());
    }
    
    public void tankDrive(double left, double right) {
        drive.setLeftRightPower(left, right);
    }
    
    public void arcadeDrive(double power, double turn) {
        drive.setLeftRightPower(power + turn, power - turn);
    }
    
    public void updateLocation() {
        double dLeft = ticksToInches(drive.getLeftEncoder()) - lastLeft;
        lastLeft += dLeft;
        double dRight = ticksToInches(drive.getRightEncoder()) - lastRight;
        lastRight += dRight;
        double d = (dLeft + dRight) / 2;
        double dx, dy, dt;
        if (MathUtils.isZero(dLeft - dRight)) {
            dx = 0;
            dy = d;
            dt = 0;
        } else if (MathUtils.isZero(dLeft + dRight)) {
            dx = 0;
            dy = 0;
            dt = (dLeft - dRight) / drive.getWheelbaseWidth();
        } else {
            double cx = ((2*dLeft)/(dLeft-dRight) - 1) * drive.getWheelbaseWidth();
            dx = cx - cx*Math.cos(d/cx);
            dy = cx*Math.sin(d/cx);
            dt = d/cx;
        }
        Angle thetaY = myLocation.direction;
        Angle thetaX = Angle.add(thetaY, Angle.EAST, Angle.AngleOrientation.COMPASS_HEADING);
        double fieldDx = dx * thetaX.getX() + dy * thetaY.getX();
        double fieldDy = dx * thetaX.getY() + dy * thetaY.getY();
        myLocation.translate(fieldDx, fieldDy);
        myLocation.direction = Angle.add(
                myLocation.direction,
                new Angle(dt, Angle.AngleUnit.RADIANS, Angle.AngleOrientation.COMPASS_HEADING),
                Angle.AngleOrientation.COMPASS_HEADING
        );
    }
    
    public void setLocation(Location l) {
        myLocation = l;
    }

    @Override
    public Location getLocation() {
        return myLocation;
    }

    @Override
    public void moveTowardLocation(Location target, double velocity) {
        Location noo = target.setOrigin(target);
        double h = noo.x;
        double k = noo.y;
        double lp, rp;
        if (MathUtils.isZero(h) && MathUtils.isZero(k)) {
            lp = 0;
            rp = 0;
        } else {
            double d2 = h*h + k*k;
            double h2 = 2 * Math.abs(h);
            double small = Math.signum(k) * (d2-h2)/(d2+h2);
            lp = h < 0 ? small : k < 0 ? -1 : 1;
            rp = h > 0 ? small : k < 0 ? -1 : 1;
            lp /= velocity;
            rp /= velocity;
        }
        drive.setLeftRightPower(lp, rp);
    }
    
    private double ticksToInches(long ticks) {
        return (ticks * drive.getEncoderTicksPerRevolution()) / (drive.getWheelDiameter() * Math.PI);
    }
    
}
