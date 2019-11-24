package org.westtorrancerobotics.lib.hardware.drive;

import org.westtorrancerobotics.lib.spline.geom.Angle;
import org.westtorrancerobotics.lib.spline.geom.Location;
import org.westtorrancerobotics.lib.spline.geom.Point;
import org.westtorrancerobotics.lib.util.MathUtils;

/**
 * Controller for a Mecanum drive with encoders on its driving wheels and an external
 * odometer. Allows for acquisition of information about the movement characteristics
 * of each wheel, as well as for control of the drive train.
 * 
 * @see MecanumWheelMovementRates
 * @see MecanumController
 * @since 1.1
 */
public class EncoderedMecanumController extends MecanumController
        implements EncoderedMecanumDrive {
    
    private final EncoderedMecanumDrive train;
    private final OdometeredDriveBase odometer;

    /**
     * Creates an {@code EncoderedMecanumController} from the supplied odometer and
     * driving wheels.
     * 
     * @param train the Mecanum drive wheel and encoder implementation
     * @param odometer the active position tracking subsystem
     * @since 1.1
     */
    public EncoderedMecanumController(EncoderedMecanumDrive train, OdometeredDriveBase odometer) {
        super(train);
        this.train = train;
        this.odometer = odometer;
    }

    private long lastLeftFront;
    private long lastRightFront;
    private long lastLeftBack;
    private long lastRightBack;
    
    private Location lastDriveLocation;
    private long lastNanos;
    
    private MecanumWheelMovementRates leftFrontRateInfo;
    private MecanumWheelMovementRates rightFrontRateInfo;
    private MecanumWheelMovementRates leftBackRateInfo;
    private MecanumWheelMovementRates rightBackRateInfo;

    /**
     * Processes the new frame of data from each the encoder of each Mecanum wheel
     * and from the odometer. Calculation is inexpensive and should be called once
     * each loop unless acquisition of encoder and odometer data is too expensive.
     * 
     * @see #getLeftBackRateInfo()
     * @see #getLeftFrontRateInfo()
     * @see #getRightBackRateInfo()
     * @see #getRightFrontRateInfo()
     * @since 1.1
     */
    public void updateRateInfo() {
        long dNanos = System.nanoTime() - lastNanos;
        lastNanos += dNanos;
        double time = dNanos / 1_000_000_000.0;
        
        long dlf = train.getLeftFrontEncoder() - lastLeftFront;
        lastLeftFront += dlf;
        double dlfIn = dlf / getEncoderTicksPerRevolution() * Math.PI * getWheelDiameter();
        long dlb = train.getLeftBackEncoder() - lastLeftBack;
        lastLeftBack += dlb;
        double dlbIn = dlb / getEncoderTicksPerRevolution() * Math.PI * getWheelDiameter();
        long drf = train.getRightFrontEncoder() - lastRightFront;
        lastRightFront += drf;
        double drfIn = drf / getEncoderTicksPerRevolution() * Math.PI * getWheelDiameter();
        long drb = train.getRightBackEncoder() - lastRightBack;
        lastRightBack += drb;
        double drbIn = drb / getEncoderTicksPerRevolution() * Math.PI * getWheelDiameter();
        
        Location loc = odometer.getLocation();
        lastDriveLocation = lastDriveLocation.setOrigin(loc);
        double oldDir = -lastDriveLocation.direction.getValue(
                Angle.AngleUnit.RADIANS, Angle.AngleOrientation.COMPASS_HEADING);
        double newDir = -lastDriveLocation.direction.getValue(
                Angle.AngleUnit.RADIANS, Angle.AngleOrientation.COMPASS_HEADING);
        double oldX = lastDriveLocation.x;
        double oldY = lastDriveLocation.y;
        if (MathUtils.isZero(oldDir - newDir)) {
            Point tRate = new Point(-oldX / time, -oldY / time);
            leftFrontRateInfo = new MecanumWheelMovementRates(tRate, dlfIn / time, true);
            leftBackRateInfo = new MecanumWheelMovementRates(tRate, dlbIn / time, false);
            rightFrontRateInfo = new MecanumWheelMovementRates(tRate, drfIn / time, false);
            rightBackRateInfo = new MecanumWheelMovementRates(tRate, drbIn / time, true);
        } else {
            double rotRadCw = -oldDir;
            double rotRelCenterX =
                    (oldY*Math.sin(rotRadCw)+Math.cos(rotRadCw)*oldX-oldX)/(Math.cos(rotRadCw)
                    - Math.sin(rotRadCw)*Math.sin(rotRadCw) + Math.cos(rotRadCw)
                    - Math.cos(rotRadCw)*Math.cos(rotRadCw) - 1);
            double rotRelCenterY = (oldX - rotRelCenterX + Math.cos(rotRadCw)*rotRelCenterX) / Math.sin(rotRadCw);
            
            Angle direction = Angle.difference(
                    new Angle(rotRelCenterX, rotRelCenterY), Angle.EAST,
                    Angle.AngleOrientation.COMPASS_HEADING
            );
            double lfdst = Math.hypot(rotRelCenterX - getWheelbaseWidth() / 2, rotRelCenterY - getFrontWheelsY());
            double lfrate = lfdst * rotRadCw / time;
            leftFrontRateInfo = new MecanumWheelMovementRates(new Point(lfrate, direction), dlfIn / time, true);
            double rfdst = Math.hypot(rotRelCenterX + getWheelbaseWidth() / 2, rotRelCenterY - getFrontWheelsY());
            double rfrate = rfdst * rotRadCw / time;
            rightFrontRateInfo = new MecanumWheelMovementRates(new Point(rfrate, direction), drfIn / time, false);
            double lbdst = Math.hypot(rotRelCenterX - getWheelbaseWidth() / 2, rotRelCenterY - getBackWheelsY());
            double lbrate = lbdst * rotRadCw / time;
            leftBackRateInfo = new MecanumWheelMovementRates(new Point(lbrate, direction), dlbIn / time, false);
            double rbdst = Math.hypot(rotRelCenterX + getWheelbaseWidth() / 2, rotRelCenterY - getBackWheelsY());
            double rbrate = rbdst * rotRadCw / time;
            rightBackRateInfo = new MecanumWheelMovementRates(new Point(rbrate, direction), drbIn / time, true);
        }
        lastDriveLocation = loc;
    }

    /**
     * Recalls the rate info of the left front wheel. Rate info is given with respect
     * to current robot position and uses the backwards difference quotient of the
     * most recent two loop measurements.
     * 
     * @return the rate info of the left front wheel
     * @see #updateRateInfo()
     * @since 1.0
     */
    public MecanumWheelMovementRates getLeftFrontRateInfo() {
        return leftFrontRateInfo;
    }

    /**
     * Recalls the rate info of the left back wheel. Rate info is given with respect
     * to current robot position and uses the backwards difference quotient of the
     * most recent two loop measurements.
     * 
     * @return the rate info of the left back wheel
     * @see #updateRateInfo()
     * @since 1.0
     */
    public MecanumWheelMovementRates getLeftBackRateInfo() {
        return leftBackRateInfo;
    }

    /**
     * Recalls the rate info of the right front wheel. Rate info is given with respect
     * to current robot position and uses the backwards difference quotient of the
     * most recent two loop measurements.
     * 
     * @return the rate info of the right front wheel
     * @see #updateRateInfo()
     * @since 1.0
     */
    public MecanumWheelMovementRates getRightFrontRateInfo() {
        return rightFrontRateInfo;
    }

    /**
     * Recalls the rate info of the back right wheel. Rate info is given with respect
     * to current robot position and uses the backwards difference quotient of the
     * most recent two loop measurements.
     * 
     * @return the rate info of the back right wheel
     * @see #updateRateInfo()
     * @since 1.0
     */
    public MecanumWheelMovementRates getRightBackRateInfo() {
        return rightBackRateInfo;
    }
    
    public static class MecanumWheelMovementRates {
        public final Point planarMovementRate;
        public final double casterRollRate;
        public final double skidRate;
        public final double driveRate;

        public MecanumWheelMovementRates(Point planarMovementRate, double driveRate,
                boolean leftFrontOrRightBack) {
            this.planarMovementRate = planarMovementRate;
            this.driveRate = driveRate;
            Point skidPlusCaster = new Point(planarMovementRate.x, planarMovementRate.y - driveRate);
            double h = skidPlusCaster.x;
            double k = skidPlusCaster.y;
            double a = (k - h) / 2 + h;
            double b = (k - h) / 2;
            a *= Math.sqrt(2);
            b *= Math.sqrt(2);
            if (leftFrontOrRightBack) {
                skidRate = a;
                casterRollRate = b;
            } else {
                skidRate = b;
                casterRollRate = a;
            }
        }
        
    }

    @Override
    public double getWheelbaseWidth() {
        return train.getWheelbaseWidth();
    }

    @Override
    public double getWheelDiameter() {
        return train.getWheelDiameter();
    }

    @Override
    public double getEncoderTicksPerRevolution() {
        return train.getEncoderTicksPerRevolution();
    }

    @Override
    public void setMotorPowers(double frontLeft, double backLeft, double frontRight, double backRight) {
        train.setMotorPowers(frontLeft, backLeft, frontRight, backRight);
    }

    @Override
    public long getLeftFrontEncoder() {
        return train.getLeftFrontEncoder();
    }

    @Override
    public long getRightFrontEncoder() {
        return train.getRightFrontEncoder();
    }

    @Override
    public long getLeftBackEncoder() {
        return train.getLeftBackEncoder();
    }

    @Override
    public long getRightBackEncoder() {
        return train.getRightBackEncoder();
    }

    @Override
    public double getFrontWheelsY() {
        return train.getFrontWheelsY();
    }

    @Override
    public double getBackWheelsY() {
        return train.getBackWheelsY();
    }
}
