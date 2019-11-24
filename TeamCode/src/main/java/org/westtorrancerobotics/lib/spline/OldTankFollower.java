package org.westtorrancerobotics.lib.spline;

import org.westtorrancerobotics.lib.spline.geom.Point;
import org.westtorrancerobotics.lib.spline.geom.Angle;
import java.io.Serializable;
import java.util.Objects;
import java.util.StringJoiner;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.westtorrancerobotics.lib.functionmath.interfaces.Function;
import org.westtorrancerobotics.lib.functionmath.interfaces.CalculusFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;
import org.westtorrancerobotics.lib.functionmath.trig.Cosine;
import org.westtorrancerobotics.lib.functionmath.trig.Arctangent;
import org.westtorrancerobotics.lib.functionmath.trig.Sine;
import org.westtorrancerobotics.lib.hardware.drive.TankDrive;
import org.westtorrancerobotics.lib.hardware.ClosedLoopController;
import org.westtorrancerobotics.lib.functionmath.Composition;
import org.westtorrancerobotics.lib.functionmath.Constant;
import org.westtorrancerobotics.lib.functionmath.Difference;
import org.westtorrancerobotics.lib.functionmath.ParametricFunction;
import org.westtorrancerobotics.lib.functionmath.Piecewise;
import org.westtorrancerobotics.lib.functionmath.PiecewiseDynamicBounds;
import org.westtorrancerobotics.lib.functionmath.Polynomial;
import org.westtorrancerobotics.lib.functionmath.PolynomialGenerator;
import org.westtorrancerobotics.lib.functionmath.Product;
import org.westtorrancerobotics.lib.functionmath.Quotient;
import org.westtorrancerobotics.lib.functionmath.SquareRoot;
import org.westtorrancerobotics.lib.functionmath.Sum;
import org.westtorrancerobotics.lib.functionmath.casters.BruteIntegral;
import org.westtorrancerobotics.lib.functionmath.casters.BruteInverse;
import org.westtorrancerobotics.lib.functionmath.casters.Inversiblify;
import org.westtorrancerobotics.lib.functionmath.interfaces.InversibleFunction;
import org.westtorrancerobotics.lib.util.StringUtils;

/**
 * A wrapper for a parametric function that uses numerical tuning coefficients and
 * a {@code TankDrive} to make the drive train follow the path provided. The path
 * provided is assumed to start at the robot's current position. Two auxiliary parametric
 * functions, one for each side of the {@code TankDrive}, are created to guide the
 * wheels with encoders, and the total distance traveled as calculated by those same
 * encoders is used to determine the distance along the spline traveled. That distance
 * is used not only to compute encoder targets, but also to compute the current directional
 * error from a gyroscope and compensate for any error found there. Once the path
 * has been traveled to its complete distance, the {@code OldTankFollower} will declare
 * itself to have finished, and it should no longer be used to drive and follow the
 * path.
 * 
 * @see TankDrive
 * @see TankFollower.Config
 * @see #TankFollower(ParametricFunction, double, double, TankDrive, TankFollower.Config)
 * @since 1.0
 */
public class OldTankFollower {

    private static final Logger LOG;
    
    static {
        LOG = Logger.getLogger(OldTankFollower.class.getName());
        LOG.setLevel(Level.OFF);
    }
    
    /**
     * Return the logger for this class. This class logs on FINER successful initialization,
     * calls to {@code follow()}, and completion of the path, while providing verbose
     * initialization success status and numerical attributes of {@code follow()} on
     * FINEST. It also logs failed initialization, as determined by NaN distance or
     * parameter calculations, on SEVERE, and logs completion of portions of the
     * calculation system of the follower before the follower has asserted itself to
     * be finished on WARNING. The logger is set to {@link Level#OFF} by default,
     * and must be changed for the log to be received.
     * 
     * @return the logger for {@code OldTankFollower.class}
     * @since 1.0
     */
    public static Logger getLogger() {
        return LOG;
    }
    
    private final ParametricFunction midSpline;
    private final ParametricFunction leftSpline;
    private final ParametricFunction rightSpline;
    private final DoubleUnaryOperator velocitySpline;
    
    //save the functions with initialization to get more time efficient
    private final Function midSplineDistanceGetter;
    private final Function leftSplineDistanceGetter;
    private final Function rightSplineDistanceGetter;
    private final Function midSplineParameterGetter;
    
    private final TankDrive driveTrain;
    private final Config config;
    private final double DST_PER_TICK; //calculated using config
    
    private final long firstLeft;
    private final long firstRight;
    private long lastLeft;
    private long lastRight;
    private double dstTravelled;
    
    private final ClosedLoopController leftPid;
    private final ClosedLoopController rightPid;
    
    private final ClosedLoopController gyroPid;
    
    private final double direction; //forward or backward
    
    private final double totalDst;
    private double lastP;
    private double lastGain;
    private double lastLeftPower;
    private double lastRightPower;
    private Angle currentDirection;
    
    /**
     * Holder for constants that are unlikely to change throughout a program.
     * These constants tune the compensation on the spline to suit the characteristics
     * of the specific drive train well, and to allow computation of meaningful data from
     * sensor values given by the drive train.
     * 
     * @since 1.0
     */
    public static class Config implements Serializable {
        
        private static final long serialVersionUID = Double.doubleToRawLongBits(1.0)
                + StringUtils.encodeAsLong(Config.class.getName()); // version 1.0
        
        /**
         * Value experimentally chosen for {@link #INTEGRAL_NUMBER_OF_SAMPLES}
         * that works well. Tends to both provide a high enough degree of accuracy
         * and avoid high processing times.
         * 
         * @since 1.0
         */
        public static final int DEFAULT_INTEGRAL_NUMBER_OF_SAMPLES = 10;
        
        /**
         * Value experimentally chosen for {@link #DISTANCE_CALC_ACCURACY}
         * that works well. Tends to both provide a high enough degree of accuracy
         * to reach targets within their tolerances and avoid high processing times.
         * 
         * @since 1.0
         */
        public static final double DEFAULT_DISTANCE_CALC_ACCURACY = 0.01;
        
        /**
         * Value experimentally chosen for {@link #VELOCITY_CALC_ACCURACY}
         * that works well. Tends to both provide a high enough degree of accuracy
         * to reach targets within their tolerances and avoid high processing times.
         * 
         * @since 1.0
         */
        public static final double DEFAULT_VELOCITY_CALC_ACCURACY = 0.001;
        
        /**
         * Value theoretically chosen for {@link #PORTION_NEXT_POWER} that works
         * well. Assumes a reasonable degree of error must be compensated for, but
         * not that so high a degree as to hinder progress of driving exists.
         * 
         * @since 1.0
         */
        public static final double DEFAULT_PORTION_NEXT_POWER = 0.6;
        
        /**
         * Value theoretically chosen for {@link #PORTION_ENCODER_ADJ} that works
         * well. Assumes a reasonable degree of error must be compensated for, but
         * not that so high a degree as to hinder progress of driving exists.
         * 
         * @since 1.0
         */
        public static final double DEFAULT_PORTION_ENCODER_ADJ = 0.2;
        
        /**
         * Value theoretically chosen for {@link #PORTION_GYRO_ADJ} that works
         * well. Assumes a reasonable degree of error must be compensated for, but
         * not that so high a degree as to hinder progress of driving exists.
         * 
         * @since 1.0
         */
        public static final double DEFAULT_PORTION_GYRO_ADJ = 0.2;
        
        /**
         * Value theoretically chosen for {@link #TURN_GAIN} that works well. A drive
         * train with low resistance to turning should use this value, as it is optimal
         * for prediction of needed speeds. However, a drive train highly resistant
         * to turning may require a larger value.
         * 
         * @since 1.0
         */
        public static final double DEFAULT_TURN_GAIN = 1;
        
        /**
         * Constant specifying the accuracy of {@link BruteIntegral}s used in the
         * follower. Higher inputs indicate more accuracy.
         * 
         * @see ParametricFunction#getDistance(int INTEGRAL_NUM_SAMPLES)
         * @see BruteIntegral#BruteIntegral(DerivableFunction, int, double)
         * @since 1.0
         */
        public final int INTEGRAL_NUMBER_OF_SAMPLES;
        
        /**
         * Constant specifying the accuracy of {@link BruteInverse}s used in the
         * follower to calculate distance. Lower inputs indicate more accuracy, and
         * all inputs must be positive.
         * 
         * @see ParametricFunction#getDistance(int INTEGRAL_NUM_SAMPLES)
         * @see BruteInverse#BruteInverse(Function, double, double, double)
         * @since 1.0
         */
        public final double DISTANCE_CALC_ACCURACY;
        
        /**
         * Constant specifying the accuracy of {@link BruteInverse}s used in the
         * follower to calculate output velocity gain. Lower inputs indicate more
         * accuracy, and all inputs must be positive.
         * 
         * @see ParametricFunction#getDistance(int INTEGRAL_NUM_SAMPLES)
         * @see BruteInverse#BruteInverse(Function, double, double, double)
         * @since 1.0
         */
        public final double VELOCITY_CALC_ACCURACY;
        
        /**
         * The maximum acceleration that the drive train supplied to this follower
         * can achieve. The follower will attempt to achieve this acceleration when
         * going from initial velocity to mid and end velocities. Acceleration is
         * defined as the second derivative of position with respect to time.
         * 
         * @see VelocitySpline#VelocitySpline
         * @since 1.0
         */
        public final double MAX_ACCELERATION;
        
        /**
         * The maximum jerk that the drive train supplied to this follower can
         * safely achieve. The follower will attempt to achieve this acceleration when
         * going from initial velocity to mid and end velocities. Jerk is
         * defined as the third derivative of position with respect to time.
         * 
         * @see VelocitySpline#VelocitySpline
         * @since 1.0
         */
        public final double MAX_JERK;
        
        /**
         * The least power with which the drive train will move. If the follower
         * applies too little power to the drive train, and expects it to move, but
         * it does not, the follower will block and be unable to continue. This power
         * is used as the minimum for any output of the {@code VelocitySpline} used
         * for power gain, even if other parameters imply that a lower power is
         * requested. Therfore, if one motor is set to this power, and the other to
         * any power, and both of those are multiplied by {@link #PORTION_NEXT_POWER}
         * (because it is possible for the drive train to have no driving error),
         * there should be measurable, continued shift in the drive train position.
         * 
         * @since 1.0
         */
        public final double MIN_MOVE_POWER;
        
        /**
         * The percentage by which the powers obtained from prediction of power needed
         * to reach the next position are multiplied. A combination of three powers,
         * summed together and specified in more detail at {@link #follow()}, are
         * used in ratios here fixed for the power outputs of each loop. This number
         * is the multiplication percentage for prediction.
         * 
         * @see OldTankFollower
         * @since 1.0
         */
        public final double PORTION_NEXT_POWER;
        
        /**
         * The percentage by which the powers obtained from compensation for error between
         * current and desired wheel position are multiplied. A combination of three powers,
         * summed together and specified in more detail at {@link #follow()}, are
         * used in ratios here fixed for the power outputs of each loop. This number
         * is the multiplication percentage for encoder adjustment.
         * 
         * @see OldTankFollower
         * @since 1.0
         */
        public final double PORTION_ENCODER_ADJ;
        
        /**
         * The percentage by which the powers obtained from compensation for error between
         * current and desired gyro direction are multiplied. A combination of three powers,
         * summed together and specified in more detail at {@link #follow()}, are
         * used in ratios here fixed for the power outputs of each loop. This number
         * is the multiplication percentage for gyroscopic adjustment.
         * 
         * @see OldTankFollower
         * @since 1.0
         */
        public final double PORTION_GYRO_ADJ;
        
        /**
         * Constant to increase or decrease the amount of turning that a drive train
         * will do. Some drive trains have high resistance to turning and need that
         * extra kick to rotate. Increasing this constant supplies that kick. If,
         * for whatever reason, less turning is needed, decreasing this constant can
         * help (although the gyro reading is more likely at fault). The robot will
         * not try to turn at all if this constant is zero, and default turning
         * (as should exist in a low-resistance turning robot) will occur if this
         * constant is one. The only power that this number changes is the prediction:
         * the control-loop driven adjustments are not affected.
         * 
         * @see #PORTION_NEXT_POWER
         * @since 1.0
         */
        public final double TURN_GAIN;
        
        /**
         * Supplier that gives the {@link ClosedLoopController}s for encoder error
         * compensation. Although this field is named "PID," there is no requirement
         * the PID is the closed loop control algorithm implemented.
         * 
         * @see OldTankFollower
         * @since 1.0
         */
        public final Supplier<ClosedLoopController> ENCODER_PID_SUPPLIER;
        
        /**
         * Supplier that gives the {@link ClosedLoopController}s for gyro error
         * compensation. Although this field is named "PID," there is no requirement
         * the PID is the closed loop control algorithm implemented.
         * 
         * @see OldTankFollower
         * @since 1.0
         */
        public final Supplier<ClosedLoopController> GYRO_PID_SUPPLIER;

        /**
         * Creates a new configuration object with the specified attributes. This is
         * a holder for constants that are unlikely to change throughout a program.
         * These constants tune the compensation on the spline to suit the characteristics
         * of the specific drive train well, and to allow computation of meaningful data from
         * sensor values given by the drive train.
         * <p>
         * The sum of {@code PORTION_NEXT_POWER}, {@code PORTION_ENCODER_ADJ}, and
         * {@code PORTION_GYRO_ADJ} shall be 1. {@code MIN_MOVE_POWER} shall be
         * between 0 and 1, exclusive.
         * <p>
         * For two {@code Config}s to be equal, their numerical parameters must be
         * equal by reference, and their {@code ClosedLoopController Supplier}s must
         * be equal by their {@code .equals} methods, which will likely return false
         * if the two {@code Supplier}s were created in different lambda expressions.
         * 
         * @param INTEGRAL_NUMBER_OF_SAMPLES {@link #INTEGRAL_NUMBER_OF_SAMPLES}
         * @param DISTANCE_CALC_ACCURACY {@link #DISTANCE_CALC_ACCURACY}
         * @param VELOCITY_CALC_ACCURACY {@link #VELOCITY_CALC_ACCURACY}
         * @param MAX_ACCELERATION {@link #MAX_ACCELERATION}
         * @param MAX_JERK {@link #MAX_JERK}
         * @param MIN_MOVE_POWER {@link #MIN_MOVE_POWER}
         * @param PORTION_NEXT_POWER {@link #PORTION_NEXT_POWER}
         * @param PORTION_ENCODER_ADJ {@link #PORTION_ENCODER_ADJ}
         * @param PORTION_GYRO_ADJ {@link #PORTION_GYRO_ADJ}
         * @param ENCODER_PID_SUPPLIER {@link #ENCODER_PID_SUPPLIER}
         * @param GYRO_PID_SUPPLIER  {@link #GYRO_PID_SUPPLIER}
         * @param TURN_GAIN {@link #TURN_GAIN}
         * @throws IllegalArgumentException if the sum of {@code PORTION_NEXT_POWER},
         *         {@code PORTION_ENCODER_ADJ}, and {@code PORTION_GYRO_ADJ} is not 1
         *         or if {@code MIN_MOVE_POWER} is not between 0 and 1, exclusive.
         * @since 1.0
         */
        public Config (
                int INTEGRAL_NUMBER_OF_SAMPLES,
                double DISTANCE_CALC_ACCURACY, double VELOCITY_CALC_ACCURACY,
                
                double MAX_ACCELERATION, double MAX_JERK,
                double MIN_MOVE_POWER,
                
                double PORTION_NEXT_POWER, double PORTION_ENCODER_ADJ,
                double PORTION_GYRO_ADJ, double TURN_GAIN,
                
                Supplier<ClosedLoopController> ENCODER_PID_SUPPLIER,
                Supplier<ClosedLoopController> GYRO_PID_SUPPLIER
        ) {
            this.INTEGRAL_NUMBER_OF_SAMPLES = INTEGRAL_NUMBER_OF_SAMPLES;
            this.DISTANCE_CALC_ACCURACY = DISTANCE_CALC_ACCURACY;
            this.VELOCITY_CALC_ACCURACY = VELOCITY_CALC_ACCURACY;
            this.MAX_ACCELERATION = MAX_ACCELERATION;
            this.MAX_JERK = MAX_JERK;
            if (MIN_MOVE_POWER <= 0 || MIN_MOVE_POWER >= 1) {
                throw new IllegalArgumentException("MIN_MOVE_POWER must be positive and less than 1.");
            }
            this.MIN_MOVE_POWER = MIN_MOVE_POWER;
            if (Math.abs(PORTION_ENCODER_ADJ + PORTION_GYRO_ADJ + PORTION_NEXT_POWER - 1) > 0.0001) {
                throw new IllegalArgumentException("Total of portions must be 1");
            }
            this.PORTION_NEXT_POWER = PORTION_NEXT_POWER;
            this.PORTION_ENCODER_ADJ = PORTION_ENCODER_ADJ;
            this.PORTION_GYRO_ADJ = PORTION_GYRO_ADJ;
            this.TURN_GAIN = TURN_GAIN;
            this.ENCODER_PID_SUPPLIER = ENCODER_PID_SUPPLIER;
            this.GYRO_PID_SUPPLIER = GYRO_PID_SUPPLIER;
        }

        /**
         * Gives a string representation of this configuration. Concatenates the
         * labeled values of each numerical field with sample outputs of the
         * {@code ClosedLoopController Supplier}s, and includes the class name.
         * 
         * @return a string representation of this {@code Config}
         * @since 1.0
         */
        @Override
        public String toString() {
            return "TankFollower.Config{"
                    + "INTEGRAL_NUMBER_OF_SAMPLES=" + INTEGRAL_NUMBER_OF_SAMPLES
                    + ", DISTANCE_CALC_ACCURACY=" + DISTANCE_CALC_ACCURACY
                    + ", MAX_ACCELERATION=" + MAX_ACCELERATION
                    + ", MAX_JERK=" + MAX_JERK
                    + ", MIN_MOVE_POWER=" + MIN_MOVE_POWER
                    + ", PORTION_NEXT_POWER=" + PORTION_NEXT_POWER
                    + ", PORTION_ENCODER_ADJ=" + PORTION_ENCODER_ADJ
                    + ", PORTION_GYRO_ADJ=" + PORTION_GYRO_ADJ
                    + ", TURN_GAIN=" + TURN_GAIN
                    + ", ENCODER_PID_SAMPLE=" + ENCODER_PID_SUPPLIER.get()
                    + ", GYRO_PID_SAMPLE=" + GYRO_PID_SUPPLIER.get()
            + '}';
        }

        /**
         * 
         * @since 1.0
         */
        @Override
        public int hashCode() {
            int hash = 7;
            hash = 37 * hash + this.INTEGRAL_NUMBER_OF_SAMPLES;
            hash = 37 * hash + (int) (Double.doubleToLongBits(this.DISTANCE_CALC_ACCURACY)
                    ^ (Double.doubleToLongBits(this.DISTANCE_CALC_ACCURACY) >>> 32));
            hash = 37 * hash + (int) (Double.doubleToLongBits(this.MAX_ACCELERATION)
                    ^ (Double.doubleToLongBits(this.MAX_ACCELERATION) >>> 32));
            hash = 37 * hash + (int) (Double.doubleToLongBits(this.MAX_JERK)
                    ^ (Double.doubleToLongBits(this.MAX_JERK) >>> 32));
            hash = 37 * hash + (int) (Double.doubleToLongBits(this.MIN_MOVE_POWER)
                    ^ (Double.doubleToLongBits(this.MIN_MOVE_POWER) >>> 32));
            hash = 37 * hash + (int) (Double.doubleToLongBits(this.PORTION_NEXT_POWER)
                    ^ (Double.doubleToLongBits(this.PORTION_NEXT_POWER) >>> 32));
            hash = 37 * hash + (int) (Double.doubleToLongBits(this.PORTION_ENCODER_ADJ)
                    ^ (Double.doubleToLongBits(this.PORTION_ENCODER_ADJ) >>> 32));
            hash = 37 * hash + (int) (Double.doubleToLongBits(this.PORTION_GYRO_ADJ)
                    ^ (Double.doubleToLongBits(this.PORTION_GYRO_ADJ) >>> 32));
            hash = 37 * hash + (int) (Double.doubleToLongBits(this.TURN_GAIN)
                    ^ (Double.doubleToLongBits(this.TURN_GAIN) >>> 32));
            hash = 37 * hash + Objects.hashCode(this.ENCODER_PID_SUPPLIER);
            hash = 37 * hash + Objects.hashCode(this.GYRO_PID_SUPPLIER);
            return hash;
        }

        /**
         * Returns if the object supplied is the same as this one.
         * For two {@code Config}s to be equal, their numerical parameters must be
         * equal by reference, and their {@code ClosedLoopController Supplier}s must
         * be equal by their {@code .equals} methods, which will likely return false
         * if the two {@code Supplier}s were created in different lambda expressions.
         * 
         * @param obj the object to compare
         * @return if the two objects are the same
         * @since 1.0
         */
        @Override
        public boolean equals(Object obj) {
            if (obj == null) {
                return this == null;
            }
            if (!(obj.getClass().equals(getClass()))) {
                return false;
            }
            Config cfg = (Config) obj;
            return cfg.DISTANCE_CALC_ACCURACY == DISTANCE_CALC_ACCURACY &&
                    cfg.INTEGRAL_NUMBER_OF_SAMPLES == INTEGRAL_NUMBER_OF_SAMPLES &&
                    cfg.MAX_ACCELERATION == MAX_ACCELERATION && cfg.MAX_JERK == MAX_JERK &&
                    cfg.MIN_MOVE_POWER == MIN_MOVE_POWER &&
                    cfg.PORTION_ENCODER_ADJ == PORTION_ENCODER_ADJ &&
                    cfg.PORTION_GYRO_ADJ == PORTION_GYRO_ADJ &&
                    cfg.PORTION_NEXT_POWER == PORTION_NEXT_POWER &&
                    cfg.TURN_GAIN == TURN_GAIN &&
                    cfg.ENCODER_PID_SUPPLIER.equals(ENCODER_PID_SUPPLIER) &&
                    cfg.GYRO_PID_SUPPLIER.equals(GYRO_PID_SUPPLIER);
        }
        
    }
    
    /**
     * Creates a new {@code TankFollower} that drives a {@code TankDrive} along the
     * path provided by {@code ParametricFunction path}. The path provided is assumed
     * to start at the robot's current position. Two auxiliary parametric functions,
     * one for each side of the {@code TankDrive}, are created to guide the wheels with
     * encoders, and the total distance traveled as calculated by those same encoders
     * is used to determine the distance along the spline traveled. That distance is used
     * not only to compute encoder targets, but also to compute the current directional
     * error from a gyroscope and compensate for any error found there. Once the path
     * has been traveled to its complete distance, the {@code TankFollower} will declare
     * itself to have finished, and it should no longer be used to drive and follow the
     * path.
     * <p>
     * The total output powers at any given time will be determined by the output of
     * {@code velocitySpline} at the given distance. Typically this will be a
     * {@link VelocitySpline} wrapped by a {@link InputOutputScalar} (due to distance control
     * loops rather than time ones), and one can be generated automatically by the other
     * constructor {@link #TankFollower(ParametricFunction, double, double, TankDrive, Config)}.
     * Another typical use of the parameter {@code velocitySpline} is to have a driver
     * control the velocity of driving.
     * <p>
     * The robot will drive forward along the path if {@link ParametricFunction#goesForward()}
     * returns true, and will assume itself to be facing toward the path, in the initial
     * location of the path. Otherwise, the robot will drive backwards along the path,
     * and assume itself to have begun at the initial point of the path and facing
     * in the direction opposite the direction of the initial location of the path,
     * where opposite is an addition or subtraction of half a turn from the known
     * angle, such that the back of the robot is facing toward the path.
     * <p>
     * The most straightforward and conventional method to create a path for a
     * {@code TankFollower} is to call {@link SplineGenerator#makeSpline}. If the
     * target is known as a relative position, with the origin at the target, then
     * the start location should be the robot position and the end location should
     * be the origin. If the target is known as a relative position, with the origin
     * at the robot, then the start location should be the origin and the end location
     * should be the target location. If both the absolute locations of the robot and
     * target are known with respect to another origin, the location of the robot
     * should be the start location, and the location of the target should be the
     * end location. The magnitudes of p should be chosen to represent the desired
     * shape of the path. If it is desired that the robot travels backwards, both
     * values of p should be negative, and the direction of the end location should
     * be made to be the opposite of what is expected if the end location requires
     * a specific direction of travel to approach due to obstacles. If the direction
     * of the end location is based on final robot orientation rather than required
     * approach, the direction of the location shall be the same with a forward or
     * a backward path.
     * <p>
     * The {@code Config} will give the follower robot specific constants that allow
     * it to most smoothly and accurately guide the robot to the intended path.
     * <p>
     * All initialization and one-time calculations that can be performed are run in
     * the constructor to minimize the time taken by each call to {@code #follow()}.
     * Because the constructor is expected to take between one and ten thousand times
     * longer to execute than a call to {@code follow()}, it may make sense to run
     * the constructor in another thread. For some desired loop times and processing
     * capacities, however, this is unnecessary because the constructor does not
     * exceed time constraints for the loop. While the constructor is running, this
     * object will remain {@code null}, so make sure in multi-threaded code that if
     * a {@code TankFollower} is used and was recently told to initialize in another
     * thread, there is a check for {@link NullPointerException}s.
     * 
     * @param path the parametric function that the follower should drive
     *             {@link SplineGenerator#makeSpline}
     * @param velocitySpline a function giving the robot velocity in terms of its traveled
     *                       distance
     * @param driveTrain the robot hardware that will move along {@code path}
     * @param c the configuration object that tunes the follower for this robot
     * @see TankDrive
     * @see ParametricFunction
     * @see VelocitySpline
     * @see Config
     * @see #follow()
     * @since 1.0
     */
    public OldTankFollower(ParametricFunction path, DoubleUnaryOperator velocitySpline, TankDrive driveTrain, Config c) {
        LOG.log(Level.FINEST, "TankFollower constructor entered.");
        path.setDistanceTolerance(c.DISTANCE_CALC_ACCURACY);
        direction = path.goesForward() ? 1 : -1;
        this.driveTrain = driveTrain;
        this.midSpline = path;
        this.config = c;
        DerivableFunction slp = new Quotient(midSpline.getY().derivative(), midSpline.getX().derivative());
        DerivableFunction sgn = new PiecewiseDynamicBounds(
                new CalculusFunction[]{new Constant(-1), new Constant(1)},
                midSpline.getX().derivative(),
                new double[]{Double.NEGATIVE_INFINITY, 0}, Double.POSITIVE_INFINITY
        );
        DerivableFunction bigX = new Composition.Derivable(
                new Composition.Derivable(slp, new Arctangent()), new Cosine());
        DerivableFunction bigY = new Composition.Derivable(
                new Composition.Derivable(slp, new Arctangent()), new Sine());
        this.leftSpline = new ParametricFunction(
                new Difference.Derivable(midSpline.getX(), new Product(
                        new Product(new Constant(driveTrain.getWheelbaseWidth())
                                , bigY), sgn)),
                new Sum.Derivable(midSpline.getY(), new Product(
                        new Product(new Constant(driveTrain.getWheelbaseWidth())
                                , bigX), sgn)),
                path.getMaxInput(), config.DISTANCE_CALC_ACCURACY
        );
        this.rightSpline = new ParametricFunction(
                new Sum.Derivable(midSpline.getX(), new Product(
                        new Product(new Constant(driveTrain.getWheelbaseWidth())
                                , bigY), sgn)),
                new Difference.Derivable(midSpline.getY(), new Product(
                        new Product(new Constant(driveTrain.getWheelbaseWidth())
                                , bigX), sgn)),
                path.getMaxInput(), config.DISTANCE_CALC_ACCURACY
        );
        LOG.log(Level.FINEST, "Left and right paths created.");
        leftSplineDistanceGetter = leftSpline.getDistance(c.INTEGRAL_NUMBER_OF_SAMPLES);
        midSplineDistanceGetter = midSpline.getDistance(c.INTEGRAL_NUMBER_OF_SAMPLES);
        rightSplineDistanceGetter = rightSpline.getDistance(c.INTEGRAL_NUMBER_OF_SAMPLES);
        midSplineParameterGetter = midSpline.getParameter(c.INTEGRAL_NUMBER_OF_SAMPLES);
        double length = midSplineDistanceGetter.get(1);
        LOG.log(Level.FINEST, "Total length computed:{0}", length);
        this.velocitySpline = velocitySpline;
        LOG.log(Level.FINEST, "VelocitySpline created.");
        makeReady();
        LOG.log(Level.FINEST, "Lazy initialization in caster functions performed.");
        firstLeft = driveTrain.getLeftEncoder();
        firstRight = driveTrain.getRightEncoder();
        lastLeft = driveTrain.getLeftEncoder();
        lastRight = driveTrain.getRightEncoder();
        dstTravelled = 0;
        leftPid = c.ENCODER_PID_SUPPLIER.get();
        rightPid = c.ENCODER_PID_SUPPLIER.get();
        totalDst = length;
        lastP = 0;
        lastGain = velocitySpline.applyAsDouble(0);
        gyroPid = c.GYRO_PID_SUPPLIER.get();
        DST_PER_TICK = Math.PI * driveTrain.getWheelDiameter() / driveTrain.getEncoderTicksPerRevolution();
        currentDirection = Angle.NORTH;
        LOG.log(Level.FINER, "TankFollower successfully constructed.");
    }
    
    /**
     * Creates a new {@code TankFollower} that drives a {@code TankDrive} along the
     * path provided by {@code ParametricFunction path}. The follower will compute
     * the current velocity of the {@code TankDrive} based on power inputs and use this
     * as its initial output velocity. It then creates a{@code VelocitySpline} using
     * this, the supplied middle velocity, and the supplied end velocity, and the
     * {@code VelocitySpline} will determine the output velocity of the follower at
     * each amount of traveled distance.
     * 
     * @param path the parametric function that the follower should drive
     *             {@link SplineGenerator#makeSpline}
     * @param endVelocity the speed for the robot when it reaches the point at parameter
     *                    one on {@code path}, as a percentage of the drive
     * @param midVelocity the speed for the robot while it cruises along its path,
     *                    as a percentage of the drive
     * @param driveTrain the robot hardware that will move along {@code path}
     * @param c the configuration object that tunes the follower for this robot
     * @see #TankFollower(ParametricFunction, DoubleUnaryOperator, TankDrive, Config)
     * @since 1.0
     */
    public OldTankFollower(ParametricFunction path, double endVelocity, double midVelocity,
            TankDrive driveTrain, Config c) {
        this(path, getVelocitySpline(path, endVelocity, midVelocity, driveTrain, c), driveTrain, c);
    }
    
    // literally just cheating "call to this must be first statement in constructor"
    private static Function getVelocitySpline(ParametricFunction path, double endVelocity,
            double midVelocity, TankDrive driveTrain, Config c) {
        if (endVelocity * midVelocity < 0) {
            throw new IllegalArgumentException("The velocity of the drive cannot pass"
                    + "through zero, or the robot will fail to move and will stall.");
        }
        int dir = path.goesForward() ? 1 : -1;
        double length = path.getDistance(c.INTEGRAL_NUMBER_OF_SAMPLES).get(1);
        double vmid = midVelocity * dir;
        if (vmid < c.MIN_MOVE_POWER) {
            vmid = c.MIN_MOVE_POWER * Math.signum(vmid);
        }
        double vinit = dir * (driveTrain.getLeftPower() + driveTrain.getRightPower()) / 2;
        if (midVelocity * vinit < 0) {
            vinit = c.MIN_MOVE_POWER * Math.signum(midVelocity);
        } else if (vinit < c.MIN_MOVE_POWER) {
            vinit = c.MIN_MOVE_POWER * Math.signum(vinit);
        }
        double vfinal = endVelocity;
        if (vfinal < c.MIN_MOVE_POWER) {
            vfinal = c.MIN_MOVE_POWER;
        }
        DerivableFunction vSpline = new OldVelocitySpline(vinit, vmid, dir * vfinal, length,
                c.MAX_ACCELERATION, c.MAX_JERK, c.VELOCITY_CALC_ACCURACY);
        return new InputOutputScalar(vSpline, c.INTEGRAL_NUMBER_OF_SAMPLES, length);
    }
    
    /**
     * Method used in initialization to activate lazy initialization of functions used
     * in this calculations done by this follower. If any of these functions return
     * {@code Double.NaN} as their inaugural output, a SEVERE message will be printed
     * to the log.
     * 
     * @since 1.0
     */
    private void makeReady() {
        if (Double.isFinite(leftSplineDistanceGetter.get(0)) &&
                Double.isFinite(rightSplineDistanceGetter.get(0)) &&
                Double.isFinite(midSplineDistanceGetter.get(0)) &&
                Double.isFinite(midSplineParameterGetter.get(0))) {
            dstTravelled = 0;
        } else {
            LOG.log(Level.SEVERE, "Conversion between parameter and distance at 0 gave NaN.");
        }
    }
    
    /**
     * Adjusts the powers on the drive train used by this {@code OldTankFollower} to
     * continue to drive along the supplied path. A combination of the needed
     * power to move to the next position and adjustments to bring any error
     * of the current position against the desired current position is used to
     * drive to the next position. Two adjustment expressions are used, one involving
     * encoders and another involving gyroscopic measurement. The three expressions
     * (one prediction and two adjustment) are multiplied by their individual
     * constants (which sum to one) and added for an overall power between -1
     * and 1.
     * <p>
     * The prediction portion of the power is obtained by differencing the current
     * parameter of {@code ParametricFunction path} with that of the last loop, and
     * determining the expected next position by adding that parameter difference to
     * the current (scaled by the power gain used). The ratio of the motor outputs
     * will reflect the ratio of future distances to travel. The two predicted motor
     * outputs are then modified if the provided value for {@code TURN_GAIN} in the
     * configuration is not one as per {@link Config#TURN_GAIN}. If {@code TURN_GAIN}
     * is one (the default value), then its existence can be ignored.
     * <p>
     * The encoder adjustment uses the output of one {@code ClosedLoopController}
     * supplied by {@code Supplier<ClosedLoopController> ENCODER_PID_SUPPLIER} from
     * the {@code Config} of this follower. The current position is set from the
     * raw encoder output of the {@code TankDrive}, and the target is calculated
     * from the wheel spline curve and fed to the {@code ClosedLoopController} in
     * units of encoder ticks. Each side of the wheel base performs this function
     * separately, with its own generated {@code ClosedLoopController} and spline
     * curve.
     * <p>
     * The encoder adjustment uses the output of one {@code ClosedLoopController}
     * supplied by {@code Supplier<ClosedLoopController> GYRO_PID_SUPPLIER} from
     * the {@code Config} of this follower. The current position is given as the
     * reading of the gyro of the {@code TankDrive} and the target is set based on
     * the direction of the {@code Location} on the path spline at the current parameter.
     * The two angles are not guaranteed to be within a half revolution of each other,
     * and the {@code ClosedLoopController} implementation provided should reflect
     * this.
     * <p>
     * The three outputs are each individually checked for magnitude in excess of
     * one, which if found, results in a scaling down of the output magnitude by its
     * absolute value. They are then individually multiplied by their portion constants
     * as supplied in the configuration. Finally, they are summed to create the final
     * motor powers, which are relayed to the {@code TankDrive}.
     * 
     * @see #TankFollower(ParametricFunction, double, double, TankDrive, TankFollower.Config)
     * @see Config
     * @since 1.0
     */
    public void follow() {
        LOG.log(Level.FINER, "Following path...");
        try {
            double currentParameter = midSplineParameterGetter.get(dstTravelled);
            double deltaP = currentParameter - lastP;
            if (deltaP == 0) {
                return;
            }
            
            long le = driveTrain.getLeftEncoder();
            long re = driveTrain.getRightEncoder();
            double deltaLeft = direction * (le - lastLeft);
            double deltaRight = direction * (re - lastRight);
            dstTravelled += (deltaLeft + deltaRight) * DST_PER_TICK / 2;
            lastLeft = le;
            lastRight = re;
            
            LOG.log(Level.FINEST, "Movement: {0} ticks left, {1} ticks right.",
                    new Object[]{           deltaLeft,      deltaRight});

            // adjustment for difference from expected current position
            double leftInchTarg = leftSplineDistanceGetter.get(currentParameter);
            double rightInchTarg = rightSplineDistanceGetter.get(currentParameter);
            double leftEncTarg = direction * leftInchTarg / DST_PER_TICK + firstLeft;
            double rightEncTarg = direction * rightInchTarg / DST_PER_TICK + firstRight;
            double leftPidOut = leftPid.getOutput(driveTrain.getLeftEncoder(), leftEncTarg);
            double rightPidOut = rightPid.getOutput(driveTrain.getRightEncoder(), rightEncTarg);
            
            LOG.log(Level.FINEST, "New targets: {0} inches left, {1} inches right.",
                    new Object[]{               leftInchTarg,    rightInchTarg});
            LOG.log(Level.FINEST, "Encoder pid: {0} % left, {1} % right.",
                    new Object[]{   leftPidOut, rightPidOut});

            // turn correction
            double radMovedCw = (deltaLeft - deltaRight) / driveTrain.getWheelbaseWidth() * direction * DST_PER_TICK;
            Angle turn = new Angle(radMovedCw, Angle.AngleUnit.RADIANS, Angle.AngleOrientation.COMPASS_HEADING);
            currentDirection = Angle.add(currentDirection, turn, Angle.AngleOrientation.COMPASS_HEADING);
            double gyroReading = currentDirection.getValue(
                    Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING);
            double gyroTarget = midSpline.getXYDir(currentParameter).direction.getValue(
                    Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING);
            if (direction == -1) {
                gyroTarget -= 180;
            }
            double gyroPidOut = gyroPid.getOutput(gyroReading, gyroTarget);
            
            LOG.log(Level.FINEST, "Gyro error: {0}", gyroTarget - gyroReading);
            LOG.log(Level.FINEST, "Gyro pid: {0}", gyroPidOut);

            // get velocity spline gain
            double gain = velocitySpline.applyAsDouble(dstTravelled);
            LOG.log(Level.FINEST, "Power gain: {0}", gain);

            // power to get to next position
            if (deltaP < config.DISTANCE_CALC_ACCURACY) {
                deltaP = config.DISTANCE_CALC_ACCURACY;
            }
            lastP = currentParameter;
            deltaP *= gain / lastGain;
            lastGain = gain;
            double nextLeftInches = leftSplineDistanceGetter.get(currentParameter + deltaP) - leftInchTarg;
            double nextRightInches = rightSplineDistanceGetter.get(currentParameter + deltaP) - rightInchTarg;
            double maxNextInches = Math.abs(nextLeftInches) > Math.abs(nextRightInches) ? nextLeftInches : nextRightInches;
            double leftPower;
            double rightPower;
            if (maxNextInches == 0) {
                leftPower = lastLeftPower;
                rightPower = lastRightPower;
            } else {
                leftPower = nextLeftInches / maxNextInches;
                rightPower = nextRightInches / maxNextInches;
                double maxPower = Math.abs(leftPower) > Math.abs(rightPower) ? leftPower : rightPower;
                double minPower = Math.abs(leftPower) < Math.abs(rightPower) ? leftPower : rightPower;
                double A = (maxPower + minPower) / (maxPower - minPower);
                double B = (maxPower + minPower) * (A + 1);
                double compensatedMin = (B / (config.TURN_GAIN + A)) - maxPower;
                if (Math.abs(leftPower) < Math.abs(rightPower)) {
                    leftPower = compensatedMin;
                } else {
                    rightPower = compensatedMin;
                }
                lastLeftPower = leftPower;
                lastRightPower = rightPower;
            }
            
            LOG.log(Level.FINEST, "Power prediction: {0} % left, {1} % right.",
                    new Object[]{                    leftPower,  rightPower});

            // combine and send outputs
            leftPidOut = Math.abs(leftPidOut) > 1 ? Math.signum(leftPidOut) : leftPidOut;
            rightPidOut = Math.abs(rightPidOut) > 1 ? Math.signum(rightPidOut) : rightPidOut;
            gyroPidOut = Math.abs(gyroPidOut) > 1 ? Math.signum(gyroPidOut) : gyroPidOut;
            double l = config.PORTION_NEXT_POWER * leftPower
                    + config.PORTION_ENCODER_ADJ * leftPidOut
                    + config.PORTION_GYRO_ADJ * gyroPidOut;
            double r = config.PORTION_NEXT_POWER * rightPower
                    + config.PORTION_ENCODER_ADJ * rightPidOut
                    - config.PORTION_GYRO_ADJ * gyroPidOut;
            if (midSpline.goesForward()) {
                driveTrain.setLeftRightPower(l * gain, r * gain);
                LOG.log(Level.FINEST, "Final powers: {0} % left, {1} % right.",
                    new Object[]{                    l * gain,   r * gain});
            } else {
                driveTrain.setLeftRightPower(r * gain, l * gain);
                LOG.log(Level.FINEST, "Final powers: {0} % left, {1} % right.",
                    new Object[]{                    r * gain,   l * gain});
            }
        } catch (IndexOutOfBoundsException ex) {
            if (!isFinished()) {
                LOG.log(Level.WARNING, "Index out of bounds before finished:", ex);
            } else {
                LOG.log(Level.FINER, "Path completed.");
            }
            dstTravelled = totalDst;
        }
    }
    
    /**
     * Tells whether the {@code OldTankFollower} has reached the end of its path. The
     * determination is made on the basis of a comparison of distance traveled and
     * total distance of the path.
     * 
     * @return true if the robot has reached the end of its path, otherwise false
     * @since 1.0
     */
    public boolean isFinished() {
        return dstTravelled >= totalDst;
    }

    /**
     * Recalls the {@code Config} supplied in the constructor of this {@code OldTankFollower}.
     * 
     * @return the {@code Config} supplied in the constructor of this {@code OldTankFollower}.
     * @see #TankFollower(ParametricFunction, double, double, TankDrive, Config) 
     * @since 1.0
     */
    public Config getConfig() {
        return config;
    }
    
    /**
     * Recalls the {@code TankDrive} supplied in the constructor of this {@code OldTankFollower}.
     * 
     * @return the {@code TankDrive} supplied in the constructor of this {@code OldTankFollower}.
     * @see #TankFollower(ParametricFunction, double, double, TankDrive, Config) 
     * @since 1.0
     */
    public TankDrive getDriveTrain() {
        return driveTrain;
    }

    /**
     * Recalls the {@code ParametricFunction} supplied in the constructor of this
     * {@code OldTankFollower}.
     * 
     * @return the {@code ParametricFunction} supplied in the constructor of this
     *         {@code OldTankFollower}.
     * @see #TankFollower(ParametricFunction, double, double, TankDrive, Config) 
     * @since 1.0
     */
    public ParametricFunction getPath() {
        return midSpline;
    }
    
    /**
     * Returns whether or not this follower is equal to the supplied object. Two
     * {@code OldTankFollower}s are equal if and only if they are of the same class,
     * and their drive train component, path component, configuration component,
     * and velocity components are equal as defined by their {@code equals()} methods.
     * 
     * @param obj the follower to compare
     * @return true if {@code obj} is the
     * @since 1.0
     */
    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return this == null;
        }
        if (!(obj.getClass().equals(getClass()))) {
            return false;
        }
        OldTankFollower tnk = (OldTankFollower) obj;
        return midSpline.equals(tnk.getPath()) && velocitySpline.equals(tnk.velocitySpline)
                && driveTrain.equals(tnk.getDriveTrain()) && config.equals(tnk.getConfig());
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = 7;
        hash = 41 * hash + Objects.hashCode(this.midSpline);
        hash = 41 * hash + Objects.hashCode(this.driveTrain);
        hash = 41 * hash + Objects.hashCode(this.config);
        return hash;
    }

    /**
     * Returns a string representation of the follower. The {@code String} will be of
 the format "OldTankFollower: Path = ([path])" without the quotes, where "[path]"
 is replaced by the string representation of the {@code ParametricFunction}
     * supplied in the constructor.
     * 
     * @return a {@code String} representing the follower
     * @since 1.0
     */
    @Override
    public String toString() {
        return "TankFollower: Path = (" + midSpline.toString() + ")";
    }
    /**
     * A function limited by the maximums and minimums of its first and second
     * derivatives. Because the output is assumed to be a velocity, the first derivative
     * assumes the name acceleration, and the second assumes the name jerk, despite their
     * conventional name assignments when an output is position. The function created
     * is limited by derivatives, and uses its initial output, end point, and a target
     * output to maintain in between in constructing itself.
     * 
     * @since 1.0
     */
    public static class OldVelocitySpline implements DerivableFunction {
 
        private final double vinit;
        private final double vmid;
        private final double vfinal;
        private final double length;
        private final double MAX_ACCELERATION;
        private final double MAX_JERK;
 
        private final double ACCURACY;
 
        private final double DX;
        private final double DY;
 
        private final DerivableFunction spline;
 
        /**
         * Creates a function limited by the maximums and minimums of its first and second
         * derivatives with domain [0, {@code length}]. The function will be such that
         * f(0) = {@code vinit}, f({@code length}) = {@code vfinal}, and for as many inputs
         * n as are possible, f(n) = {@code vmid}. The range of f'(x) at any x in the
         * domain [0, {@code length}] will be included in [-{@code MAX_ACCELERATION},
         * {@code MAX_ACCELERATION}] and the range of f''(x) at any x in the domain
         * [0, {@code length}] will be included in [-{@code MAX_JERK}, {@code MAX_JERK}].
         * If this is impossible, a linear function including the points (0, {@code vinit})
         * and ({@code length}, {@code vfinal}) will be returned. If {@code length}
         * is zero, an {@link ArithmeticException} will be thrown. The function is not
         * guaranteed to have exact results if {@code length} is too small for more
         * than a single input to exist f(n) = {@code vmid}, and {@code ACCURACY}
         * makes the function determine its outputs more precisely with a smaller positive
         * value. Additional calculations take place only during initialization.
         * 
         * @param vinit output desired in the constructed function at x = 0
         * @param vmid output desired in the constructed function for as many input values
         *             as possible
         * @param vfinal output desired in the constructed function at x = {@code length}
         * @param length specification of the function's valid domain and of the input at
         *               which {@code vfinal must output}
         * @param MAX_ACCELERATION limit of the function's first derivative's magnitude
         * @param MAX_JERK limit of the function's first derivative's magnitude
         * @param ACCURACY limit of the function's second derivative's magnitude
         * @since 1.0
         */
        public OldVelocitySpline(double vinit, double vmid, double vfinal, double length,
                double MAX_ACCELERATION, double MAX_JERK, double ACCURACY) {
            this.vinit = vinit;
            this.vmid = vmid;
            this.vfinal = vfinal;
            this.length = length;
            this.MAX_ACCELERATION = MAX_ACCELERATION;
            this.MAX_JERK = MAX_JERK;
            this.ACCURACY = ACCURACY;
            DX = MAX_ACCELERATION / MAX_JERK;
            DY = DX*MAX_ACCELERATION/2;
            DerivableFunction splineHolder;
            InterpolateSpeeds front = new InterpolateSpeeds(true, 0, vinit, vmid);
            InterpolateSpeeds back = new InterpolateSpeeds(false, length, vmid, vfinal);
            double diff = front.getXLength() + back.getXLength() - length;
            if (diff <= 0) {
                splineHolder = new Piecewise(new CalculusFunction[]{
                    front.getFunction(),
                    new Constant(vmid),
                    back.getFunction()
                }, new double[]{Double.NEGATIVE_INFINITY, front.getFinalPt().x,
                    back.getInitPt().x}, Double.POSITIVE_INFINITY);
            } else {
                try {
                    if (vinit <= vmid && vmid >= vfinal) {
                        splineHolder = new Max(vinit, vfinal, length);
                    } else if (vinit >= vmid && vmid <= vfinal) {
                        splineHolder = new Min(vinit, vfinal, length);
                    } else {
                        splineHolder = new ConnectBySlope(vinit, vmid, vfinal, length);
                    }
                } catch (ArithmeticException ex) {
                    splineHolder = PolynomialGenerator.generateLine(0, length, vinit, vfinal);
                }
            }
            this.spline = splineHolder;
        }
 
        /**
         * {@inheritDoc}
         * @see #VelocitySpline(double, double, double, double, double, double, double)
         * @since 1.0
         */
        @Override
        public double get(double x) {
            return spline.get(x);
        }
 
        /**
         * {@inheritDoc}
         * @since 1.0
         */
        @Override
        public DerivableFunction derivative() {
            return spline.derivative();
        }
 
        /**
         * A function that has the maximum integral allowed by its derivative and output
         * restrictions. The output is restricted at initial and final points by constructor
         * parameters. The derivative restrictions are the same as those of the enclosing
         * instance.
         * 
         * @see #Max(double, double, double)
         * @since 1.0
         */
        public class Max implements CalculusFunction {
 
            private final CalculusFunction me;
 
            /**
             * Creates a function that has the maximum integral allowed by its derivative
             * and output restrictions. The output is restricted to f(0) = {@code vinit}
             * and f({@code length}) = {@code vfinal}. The derivative restrictions are
             * the same as those of the enclosing instance.
             * 
             * @param vinit output restriction for x = 0
             * @param vfinal output restriction for x = {@code length}
             * @param length specification of the function's valid domain and of the input at
             *               which {@code vfinal must output}
             * @throws ArithmeticException if no such function can be made
             * @see VelocitySpline#VelocitySpline(double, double, double, double, double, double, double)
             * @since 1.0
             */
            public Max(double vinit, double vfinal, double length) {
                Polynomial lesserInput;
                Polynomial greaterInput;
                if (vfinal < vinit) {
                    lesserInput = new Polynomial(1, -vinit);
                    greaterInput = new Polynomial(1, -vfinal);
                } else {
                    greaterInput = new Polynomial(1, -vinit);
                    lesserInput = new Polynomial(1, -vfinal);
                }
                Function f = new SumTwoPartPiecewise(
                        new AddableTwoPartPiecewise(
                                new Product(new SquareRoot(), new Constant(2 * DX / Math.sqrt(2 * DY))),
                                new Polynomial(1 / MAX_ACCELERATION, DX),
                                lesserInput
                        ),
                        new AddableTwoPartPiecewise(
                                new Product(new SquareRoot(), new Constant(2 * DX / Math.sqrt(2 * DY))),
                                new Polynomial(1 / MAX_ACCELERATION, DX),
                                greaterInput
                        ), 2 * DX, Math.max(vfinal, vinit), vmid
                ).inverse();
                double midy;
                try {
                    midy = f.get(length);
                } catch (IndexOutOfBoundsException ex) {
                    throw new ArithmeticException();
                }
                InterpolateSpeeds a = new InterpolateSpeeds(true, 0, vinit, midy);
                InterpolateSpeeds z = new InterpolateSpeeds(false, length, midy, vfinal);
                double midx = (a.getFinalPt().x + z.getInitPt().x) / 2;
                me = new Piecewise(new CalculusFunction[]{
                    a.getFunction(),
                    z.getFunction()
                }, new double[]{Double.NEGATIVE_INFINITY, midx}, Double.POSITIVE_INFINITY);
            }
 
            /**
             * {@inheritDoc}
             * @since 1.0
             */
            @Override
            public CalculusFunction derivative() {
                return me.derivative();
            }
 
            /**
             * {@inheritDoc}
             * @since 1.0
             */
            @Override
            public CalculusFunction integral() {
                return me.integral();
            }
 
            /**
             * {@inheritDoc}
             * @see #Max(double, double, double)
             * @since 1.0
             */
            @Override
            public double get(double x) {
                return me.get(x);
            }
 
            /**
             * {@inheritDoc}
             * @since 1.0
             */
            @Override
            public String toString() {
                return me.toString();
            }
 
            /**
             * {@inheritDoc}
             * @since 1.0
             */
            @Override
            public boolean equals(Object obj) {
                if (obj == null) {
                    return this == null;
                }
                if (!(obj.getClass().equals(getClass()))) {
                    return false;
                }
                Max max = (Max) obj;
                return max.me.equals(me);
            }
 
            /**
             * {@inheritDoc}
             * @since 1.0
             */
            @Override
            public int hashCode() {
                int hash = 3;
                hash = 97 * hash + Objects.hashCode(this.me);
                return hash;
            }
        }
 
        /**
         * A function that has the minimum integral allowed by its derivative and output
         * restrictions. The output is restricted at initial and final points by constructor
         * parameters. The derivative restrictions are the same as those of the enclosing
         * instance.
         * 
         * @see #Min(double, double, double)
         * @since 1.0
         */
        public class Min implements CalculusFunction {
 
            private final CalculusFunction me;
 
            /**
             * Creates a function that has the minimum integral allowed by its derivative
             * and output restrictions. The output is restricted to f(0) = {@code vinit}
             * and f({@code length}) = {@code vfinal}. The derivative restrictions are
             * the same as those of the enclosing instance.
             * 
             * @param vinit output restriction for x = 0
             * @param vfinal output restriction for x = {@code length}
             * @param length specification of the function's valid domain and of the input at
             *               which {@code vfinal must output}
             * @throws ArithmeticException if no such function can be made
             * @see VelocitySpline#VelocitySpline(double, double, double, double, double, double, double)
             * @since 1.0
             */
            public Min(double vinit, double vfinal, double length) {
                Polynomial lesserInput;
                Polynomial greaterInput;
                if (vfinal < vinit) {
                    lesserInput = new Polynomial(-1, vfinal);
                    greaterInput = new Polynomial(-1, -vinit);
                } else {
                    greaterInput = new Polynomial(-1, vfinal);
                    lesserInput = new Polynomial(-1, vinit);
                }
                Function f = new SumTwoPartPiecewise(
                        new AddableTwoPartPiecewise(
                                new Product(new SquareRoot(), new Constant(2 * DX / Math.sqrt(2 * DY))),
                                new Polynomial(1 / MAX_ACCELERATION, DX),
                                lesserInput
                        ),
                        new AddableTwoPartPiecewise(
                                new Product(new SquareRoot(), new Constant(2 * DX / Math.sqrt(2 * DY))),
                                new Polynomial(1 / MAX_ACCELERATION, DX),
                                greaterInput
                        ), 2 * DX, vmid, Math.min(vfinal, vinit)
                ).inverse();
                double midy;
                try {
                    midy = f.get(length);
                } catch (IndexOutOfBoundsException ex) {
                    throw new ArithmeticException();
                }
                InterpolateSpeeds a = new InterpolateSpeeds(true, 0, vinit, midy);
                InterpolateSpeeds z = new InterpolateSpeeds(false, length, midy, vfinal);
                double midx = (a.getFinalPt().x + z.getInitPt().x) / 2;
                me = new Piecewise(new CalculusFunction[]{
                    a.getFunction(),
                    z.getFunction()
                }, new double[]{Double.NEGATIVE_INFINITY, midx}, Double.POSITIVE_INFINITY);
            }
 
            /**
             * {@inheritDoc}
             * @since 1.0
             */
            @Override
            public CalculusFunction derivative() {
                return me.derivative();
            }
 
            /**
             * {@inheritDoc}
             * @since 1.0
             */
            @Override
            public CalculusFunction integral() {
                return me.integral();
            }
 
            /**
             * {@inheritDoc}
             * @see #Min(double, double, double)
             * @since 1.0
             */
            @Override
            public double get(double x) {
                return me.get(x);
            }
 
            /**
             * {@inheritDoc}
             * @since 1.0
             */
            @Override
            public String toString() {
                return me.toString();
            }
 
            /**
             * {@inheritDoc}
             * @since 1.0
             */
            @Override
            public boolean equals(Object obj) {
                if (obj == null) {
                    return this == null;
                }
                if (!(obj.getClass().equals(getClass()))) {
                    return false;
                }
                Min min = (Min) obj;
                return min.me.equals(me);
            }
 
            /**
             * {@inheritDoc}
             * @since 1.0
             */
            @Override
            public int hashCode() {
                int hash = 7;
                hash = 53 * hash + Objects.hashCode(this.me);
                return hash;
            }
        }
 
        /**
         * A function whose difference from a constant has minimum integral allowed by
         * its derivative and output restrictions. The output is restricted at initial
         * and final points by constructor parameters, and the constant to which the distance
         * is minimized is also there specified. The derivative restrictions are the
         * same as those of the enclosing instance.
         * 
         * @see #ConnectBySlope(double, double, double, double)
         * @since 1.0
         */
        public class ConnectBySlope implements CalculusFunction {
 
            private final CalculusFunction me;
 
            /**
             * Creates a function whose difference from a constant has minimum integral
             * allowed by its derivative and output restrictions. The output is restricted
             * to f(0) = {@code vinit} and f({@code length}) = {@code vfinal}. The derivative
             * restrictions are the same as those of the enclosing instance. {@code vmid}
             * specifies the constant for which the integral of the difference of it and
             * this function will be minimized. It is assumed that the inputs to this
             * function cannot be connected using two {@link InterpolateSpeeds} and a
             * constant function with output {@code vmid}. If it is impossible for them
             * to be connected with a single {@link InterpolateSpeeds}, then no function
             * can be made to satisfy the constraints and an {@link IndexOutOfBoundsException}
             * will be thrown.
             * 
             * @param vinit output restriction for x = 0
             * @param vmid constant toward which this function attempts to near
             * @param vfinal output restriction for x = {@code length}
             * @param length specification of the function's valid domain and of the input at
             *               which {@code vfinal must output}
             * @throws ArithmeticException if no such function can be made
             * @see VelocitySpline#VelocitySpline(double, double, double, double, double, double, double)
             * @since 1.0
             */
            public ConnectBySlope(double vinit, double vmid, double vfinal, double length) {
                boolean goesUp = vfinal - vinit > 0;
                double goesUpSgn = goesUp ? 1 : -1;
                double midx = length / 2;
                Function fDy = (double x) -> {
                    InterpolateSpeeds l = new InterpolateSpeeds(true, 0, vinit, vmid + x);
                    InterpolateSpeeds r = new InterpolateSpeeds(false, length, vmid - x, vfinal);
                    return l.getFunction().get(midx) - r.getFunction().get(midx);
                };
                double myDy;
                try {
                    myDy = new Inversiblify.NonDerivable(fDy, ACCURACY / 10,
                            -Math.abs(vfinal - vinit) / 2, Math.abs(vfinal - vinit) / 2)
                            .inverse().get(0);
                } catch (IndexOutOfBoundsException ex) {
                    throw new ArithmeticException();
                }
                InterpolateSpeeds leftSide = new InterpolateSpeeds(true, 0, vinit, vmid + myDy * goesUpSgn);
                InterpolateSpeeds rightSide = new InterpolateSpeeds(false, length, vmid - myDy * goesUpSgn, vfinal);
                me = new Piecewise(new CalculusFunction[]{
                            leftSide.getFunction(),
                            rightSide.getFunction()
                        }, Double.NEGATIVE_INFINITY, midx, Double.POSITIVE_INFINITY);
            }
 
            /**
             * {@inheritDoc}
             * @since 1.0
             */
            @Override
            public CalculusFunction derivative() {
                return me.derivative();
            }
 
            /**
             * {@inheritDoc}
             * @since 1.0
             */
            @Override
            public CalculusFunction integral() {
                return me.integral();
            }
 
            /**
             * {@inheritDoc}
             * @see #ConnectBySlope(double, double, double, double)
             * @since 1.0
             */
            @Override
            public double get(double x) {
                return me.get(x);
            }
 
            /**
             * {@inheritDoc}
             * @since 1.0
             */
            @Override
            public String toString() {
                return me.toString();
            }
 
            /**
             * {@inheritDoc}
             * @since 1.0
             */
            @Override
            public boolean equals(Object obj) {
                if (obj == null) {
                    return this == null;
                }
                if (!(obj.getClass().equals(getClass()))) {
                    return false;
                }
                ConnectBySlope cbs = (ConnectBySlope) obj;
                return cbs.me.equals(me);
            }
 
            /**
             * {@inheritDoc}
             * @since 1.0
             */
            @Override
            public int hashCode() {
                int hash = 7;
                hash = 29 * hash + Objects.hashCode(this.me);
                return hash;
            }
        }
 
        private class AccJerkParabola {
 
            private final Polynomial parabola;
            private final Point start, end;
            private final double multiplier;
 
            public AccJerkParabola(double xi, double yi, boolean positive) {
                multiplier = MAX_JERK / 2 * (positive ? 1 : -1);
                parabola =  new Polynomial(multiplier, -2 * xi  * multiplier, xi * xi * multiplier + yi);
                double dx2 = DY * (positive ? 1 : -1) + yi;
                start = new Point(xi - DX, dx2);
                end   = new Point(xi + DX, dx2);
            }
 
            public Polynomial getParabola() {
                return parabola;
            }
 
            public Point getStart() {
                return start;
            }
 
            public Point getEnd() {
                return end;
            }
 
            public double getDiffTo(double dX) {
                return Math.sqrt(Math.abs(dX / multiplier));
            }
 
            @Override
            public String toString() {
                return parabola.toString() + "\n" + start.toString() + "\n" + end.toString();
            }
 
            @Override
            public boolean equals(Object obj) {
                if (obj == null) {
                    return this == null;
                }
                if (!(obj.getClass().equals(getClass()))) {
                    return false;
                }
                AccJerkParabola ajp = (AccJerkParabola) obj;
                return ajp.parabola.equals(parabola) && ajp.start.equals(start);
            }
 
            @Override
            public int hashCode() {
                int hash = 7;
                hash = 47 * hash + Objects.hashCode(this.parabola);
                hash = 47 * hash + Objects.hashCode(this.start);
                return hash;
            }
        }
 
        /**
         * A  function with a specified endpoint, output of one end, and with constrained
         * derivatives. The endpoint and output of one end will be specified by constructor
         * parameters. The derivative restrictions are the same as those of the enclosing
         * instance.
         */
        public class InterpolateSpeeds {
 
            private final Piecewise function;
            private final double xLength;
            private final Point initPt;
            private final Point finalPt;
 
            /**
             * Creates a function with a specified endpoint, output of one end, and with
             * constrained derivatives. If {@code initial} is true, then the two constraint
             * points of the function will be ({@code x}, {@code inity}) and (?, {@code endy});
             * otherwise those points will be (?, {@code inity}) and ({@code x}, {@code endy}),
             * in domain-sorted order. The x-coordinate ? will be assigned to the nearest
             * value to {@code x} that allow the function to satisfy the derivative
             * constraints of the enclosing instance.
             * 
             * @param initial whether or not the provided x-coordinate is of the initial
             *                or final point
             * @param x the x coordinate of the point specified by {@code initial}
             * @param inity the y coordinate of the initial point
             * @param endy the y coordinate of the final point
             * @see VelocitySpline#VelocitySpline(double, double, double, double, double, double, double)
             * @since 1.0
             */
            public InterpolateSpeeds(boolean initial, double x, double inity, double endy) {
                boolean goesUp = endy - inity > 0;
                double goesUpSgn = goesUp ? 1 : -1;
                if (2*DY < Math.abs(inity - endy)) {
                    // will need a straight section
                    if (initial) {
                        // (x, inity) -> (?, endy)
                        // leftmost segment
                        AccJerkParabola pt1 = new AccJerkParabola(x, inity, goesUp);
                        // middle segment
                        double midDy = endy - inity - 2 * DY * goesUpSgn;
                        double slp = MAX_ACCELERATION;
                        double midDx = midDy / slp * goesUpSgn;
                        double pt1x = pt1.getEnd().x;
                        double pt1y = pt1.getEnd().y;
                        Polynomial line = PolynomialGenerator.generateLine(pt1x, pt1y, pt1x + midDx, pt1y + midDy);
                        // rightmost segment
                        AccJerkParabola pt2 = new AccJerkParabola(pt1x + midDx + DX, endy, !goesUp);
                        function = new Piecewise(new CalculusFunction[]{
                            pt1.getParabola(),
                            line,
                            pt2.getParabola()
                        }, new double[]{Double.NEGATIVE_INFINITY, pt1.getEnd().x, pt2.getStart().x}, Double.POSITIVE_INFINITY);
                        xLength = 2 * DX + midDx;
                        initPt = new Point(x, inity);
                        finalPt = new Point(x + xLength, endy);
                    } else {
                        /// (?, inity) -> (x, endy)
                        // rightmost segment
                        AccJerkParabola pt1 = new AccJerkParabola(x, endy, !goesUp);
                        // middle segment
                        double midDy = endy - inity - 2 * DY * goesUpSgn; // positive direction
                        double slp = MAX_ACCELERATION;
                        double midDx = midDy / slp * goesUpSgn; // positive direction
                        double pt1x = pt1.getStart().x;
                        double pt1y = pt1.getStart().y;
                        Polynomial line = PolynomialGenerator.generateLine(pt1x, pt1y, pt1x - midDx, pt1y - midDy);
                        // leftmost segment
                        AccJerkParabola pt2 = new AccJerkParabola(pt1x - midDx - DX, inity, goesUp);
                        function = new Piecewise(new CalculusFunction[]{
                            pt2.getParabola(),
                            line,
                            pt1.getParabola()
                        }, new double[]{Double.NEGATIVE_INFINITY, pt2.getEnd().x, pt1.getStart().x}, Double.POSITIVE_INFINITY);
                        xLength = 2 * DX + midDx;
                        initPt = new Point(x - xLength, inity);
                        finalPt = new Point(x, endy);
                    }
                } else {
                    // will not need a straight section
                    if (initial) {
                        AccJerkParabola pt1 = new AccJerkParabola(x, inity, goesUp);
                        double dx = pt1.getDiffTo((endy - inity)/2);
                        AccJerkParabola pt2 = new AccJerkParabola(x + 2*dx, endy, !goesUp);
                        function = new Piecewise(new CalculusFunction[]{
                            pt1.getParabola(),
                            pt2.getParabola()
                        }, new double[]{Double.NEGATIVE_INFINITY, x + dx}, Double.POSITIVE_INFINITY);
                        xLength = 2 * dx;
                        initPt = new Point(x, inity);
                        finalPt = new Point(x + xLength, endy);
                    } else {
                        AccJerkParabola pt1 = new AccJerkParabola(x, endy, !goesUp);
                        double dx = pt1.getDiffTo((endy - inity)/2);
                        AccJerkParabola pt2 = new AccJerkParabola(x - 2*dx, inity, goesUp);
                        function = new Piecewise(new CalculusFunction[]{
                            pt2.getParabola(),
                            pt1.getParabola()
                        }, new double[]{Double.NEGATIVE_INFINITY, x - dx}, Double.POSITIVE_INFINITY);
                        xLength = 2 * dx;
                        initPt = new Point(x - xLength, endy);
                        finalPt = new Point(x, inity);
                    }
                }
            }
 
            /**
             * Gives the actual function generated as a part of this object.
             * 
             * @return the function denoted by this object
             * @see #InterpolateSpeeds(boolean, double, double, double)
             * @since 1.0
             */
            public Piecewise getFunction() {
                return function;
            }
 
            /**
             * Gives the difference between the x-coordinates of the initial and final
             * points on this object. This value is the absolute difference between ?
             * and {@code x}.
             * 
             * @return the difference between the x-coordinates of the initial and final
             *         points
             * @see #InterpolateSpeeds(boolean, double, double, double)
             * @since 1.0
             */
            public double getXLength() {
                return xLength;
            }
 
            /**
             * Gives the calculated or specified initial point of this object.
             * 
             * @return the calculated or specified initial point of this object
             * @see #InterpolateSpeeds(boolean, double, double, double)
             * @since 1.0
             */
            public Point getInitPt() {
                return initPt;
            }
 
            /**
             * Gives the calculated or specified final point of this object.
             * 
             * @return the calculated or specified final point of this object
             * @see #InterpolateSpeeds(boolean, double, double, double)
             * @since 1.0
             */
            public Point getFinalPt() {
                return finalPt;
            }
 
            /**
             * Returns the string representation of the function generated by this object.
             * 
             * @return the string representation of the function generated by this object
             * @see Function#toString()
             * @see #InterpolateSpeeds(boolean, double, double, double)
             * @since 1.0
             */
            @Override
            public String toString() {
                return function.toString();
            }
 
            /**
             * Returns whether or not this object is equal to the one specified. Two
             * {@code InterpolateSpeeds} are equal if they are of the same class and if
             * their underlying functions are equal.
             * 
             * @param obj the object to compare
             * @return true if this object is the same as the one specified
             * @since 1.0
             */
            @Override
            public boolean equals(Object obj) {
                if (obj == null) {
                    return this == null;
                }
                if (!(obj.getClass().equals(getClass()))) {
                    return false;
                }
                InterpolateSpeeds isp = (InterpolateSpeeds) obj;
                return isp.function.equals(function);
            }
 
            /**
             * {@inheritDoc}
             * @since 1.0
             */
            @Override
            public int hashCode() {
                int hash = 7;
                hash = 43 * hash + Objects.hashCode(this.function);
                return hash;
            }
 
        }
 
        private class AddableTwoPartPiecewise {
 
            public final DerivableFunction part1, part2;
            public final DerivableFunction input;
 
            public AddableTwoPartPiecewise(DerivableFunction part1, DerivableFunction part2, DerivableFunction input) {
                this.part1 = part1;
                this.part2 = part2;
                this.input = input;
            }
 
            @Override
            public String toString() {
                return new Composition.Derivable(part1, input).toString() + "\n" +
                        new Composition.Derivable(part2, input).toString();
            }
 
            @Override
            public boolean equals(Object obj) {
                if (obj == null) {
                    return this == null;
                }
                if (!(obj.getClass().equals(getClass()))) {
                    return false;
                }
                AddableTwoPartPiecewise atp = (AddableTwoPartPiecewise) obj;
                return atp.part1.equals(part1) && atp.part2.equals(part2) && atp.input.equals(input);
            }
 
            @Override
            public int hashCode() {
                int hash = 7;
                hash = 71 * hash + Objects.hashCode(this.part1);
                hash = 71 * hash + Objects.hashCode(this.part2);
                hash = 71 * hash + Objects.hashCode(this.input);
                return hash;
            }
 
        }
 
        private class SumTwoPartPiecewise implements InversibleFunction {
 
            private final AddableTwoPartPiecewise part1, part2;
            private final double bound, max, min;
 
            public SumTwoPartPiecewise(AddableTwoPartPiecewise part1, AddableTwoPartPiecewise part2, double bound,
                    double min, double max) {
                this.part1 = part1;
                this.part2 = part2;
                this.bound = bound;
                this.min = min;
                this.max = max;
            }
 
            @Override
            public InversibleFunction inverse() {
                return new Inversiblify.NonDerivable(this, ACCURACY, min, max).inverse();
            }
 
            // assume part1.input is always less than part2.input
            @Override
            public double get(double x) {
                if (part2.input.get(x) < bound) {
                    return part2.part1.get(part2.input.get(x)) + part1.part1.get(part1.input.get(x));
                } else if (part1.input.get(x) < bound && bound <= part2.input.get(x)) {
                    return part1.part1.get(part1.input.get(x)) + part2.part2.get(part2.input.get(x));
                } else if (bound <= part1.input.get(x)) {
                    return part1.part2.get(part1.input.get(x)) + part2.part2.get(part2.input.get(x));
                } else {
                    throw new ArithmeticException("Domain exceeded.");
                }
            }
 
            @Override
            public String toString() {
                String part2In = part2.input.toString();
                String part1In = part1.input.toString();
                String bnd = StringUtils.formatDouble(bound) + " ";
                StringJoiner joiner = new StringJoiner("");
                String[] elements = new String[]{
                        "\\left\\{", StringUtils.formatDouble(min), "<=", part2In, "<", bnd, ":",
                        new Composition.Derivable(part2.input, part2.part1).toString(),
                        "+", new Composition.Derivable(part1.input, part1.part1).toString(),
                        ",", part1In, "<", bnd, "<= ", part2In, ":",
                        new Composition.Derivable(part1.input, part1.part1).toString(),
                        "+", new Composition.Derivable(part2.input, part2.part2).toString(),
                        ",", bnd, "<= ", part1In, ":",
                        new Composition.Derivable(part1.input, part1.part2).toString(),
                        "+", new Composition.Derivable(part2.input, part2.part2).toString(),
                        "\\right\\}\\left\\{", StringUtils.formatDouble(min), "<", part1In, "\\right\\}"
                };
                for (CharSequence cs: elements) {
                    joiner.add(cs);
                }
                return joiner.toString();
            }
 
            @Override
            public boolean equals(Object obj) {
                if (obj == null) {
                    return this == null;
                }
                if (!(obj.getClass().equals(getClass()))) {
                    return false;
                }
                SumTwoPartPiecewise stp = (SumTwoPartPiecewise) obj;
                return stp.part1.equals(part1) && stp.part2.equals(part2) && stp.bound == bound;
            }
 
            @Override
            public int hashCode() {
                int hash = 5;
                hash = 47 * hash + Objects.hashCode(this.part1);
                hash = 47 * hash + Objects.hashCode(this.part2);
                hash = 47 * hash + (int) (Double.doubleToLongBits(this.bound) ^ (Double.doubleToLongBits(this.bound) >>> 32));
                return hash;
            }
        }
 
        /**
         * {@inheritDoc}
         * @since 1.0
         */
        @Override
        public String toString() {
            return spline.toString();
        }
 
        /**
         * {@inheritDoc}
         * @since 1.0
         */
        @Override
        public boolean equals(Object obj) {
            if (obj == null) {
                return this == null;
            }
            if (!(obj.getClass().equals(getClass()))) {
                return false;
            }
            OldVelocitySpline vsp = (OldVelocitySpline) obj;
            return vsp.spline.equals(spline);
        }
 
        /**
         * {@inheritDoc}
         * @since 1.0
         */
        @Override
        public int hashCode() {
            int hash = 3;
            hash = 23 * hash + Objects.hashCode(this.spline);
            return hash;
        }
    }
}