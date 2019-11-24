package org.westtorrancerobotics.lib.spline;

import org.westtorrancerobotics.lib.spline.geom.Location;
import org.westtorrancerobotics.lib.functionmath.ParametricFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.CalculusFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.Function;
import org.westtorrancerobotics.lib.hardware.drive.OdometeredDriveBase;

public class OdometerFollower {
    
    private final OdometeredDriveBase train;
    private final ParametricFunction path;
    private final Config config;
    private final long startNanos;
    private final long numNanos;
    private final Function pGetter;
    private final CalculusFunction dst;
    private final VelocitySpline vel;
    private final double maxDst;
    private double lastP;
    private double lastV;
    
    public static class Config {
        public final double MAX_VELOCITY;
        public final double MAX_ACCELERATION;
        public final double MAX_JERK;
        public final int INTEGRAL_NUM_SAMPLES;
        public final double ACCURACY;

        public Config(double MAX_VELOCITY, double MAX_ACCELERATION, double MAX_JERK, int INTEGRAL_NUM_SAMPLES, double ACCURACY) {
            this.MAX_VELOCITY = MAX_VELOCITY;
            this.MAX_ACCELERATION = MAX_ACCELERATION;
            this.MAX_JERK = MAX_JERK;
            this.INTEGRAL_NUM_SAMPLES = INTEGRAL_NUM_SAMPLES;
            this.ACCURACY = ACCURACY;
        }
    }
    
    public OdometerFollower(OdometeredDriveBase train, ParametricFunction path, Config config) {
        this.train = train;
        this.path = path;
        this.config = config;
        path.setDistanceTolerance(config.ACCURACY);
        maxDst = path.getDistance(config.INTEGRAL_NUM_SAMPLES).get(path.getMaxInput());
        vel = new VelocitySpline(config.MAX_VELOCITY, maxDst, config.MAX_ACCELERATION, config.MAX_JERK);
        dst = vel.integral();
        pGetter = path.getParameter(config.INTEGRAL_NUM_SAMPLES);
        numNanos = (long) (vel.getTotalTime() * 1e9);
        startNanos = System.nanoTime();
        lastP = 0;
    }
    
    public void follow() {
        double t = System.nanoTime() - startNanos;
        t /= 1e9;
        double p = pGetter.get(dst.get(t));
        double dp = p - lastP;
        lastP += dp;
        double v = vel.get(t);
        Location target = path.getXYDir(p + dp * (v / lastV) / maxDst);
        lastV = v;
        train.moveTowardLocation(target, v);
    }
    
    public boolean isFinished() {
        return startNanos + numNanos < System.nanoTime();
    }
    
    public Config getConfig() {
        return config;
    }
    
    public OdometeredDriveBase getDriveTrain() {
        return train;
    }
    
    public ParametricFunction getPath() {
        return path;
    }
    
}
