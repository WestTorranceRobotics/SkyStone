package org.westtorrancerobotics.lib.functionmath;

import org.westtorrancerobotics.lib.functionmath.interfaces.Function;
import org.westtorrancerobotics.lib.functionmath.casters.Calculify;
import org.westtorrancerobotics.lib.functionmath.casters.Inversiblify;
import org.westtorrancerobotics.lib.functionmath.interfaces.CalculusFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;
import java.util.HashMap;
import java.util.Objects;
import org.westtorrancerobotics.lib.functionmath.casters.BruteIntegral;
import org.westtorrancerobotics.lib.functionmath.casters.BruteInverse;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableInversibleFunction;
import org.westtorrancerobotics.lib.spline.geom.Location;
import org.westtorrancerobotics.lib.spline.geom.Angle;
import org.westtorrancerobotics.lib.spline.geom.Point;

/**
 * A function that gives {@code x} and {@code y} coordinates when supplied time
 * parameters. A {@code ParametricFunction} is not a {@code Function}, and although
 * the time parameter is typically used as time, it is not required that it be.
 * A {@code Function} for each the x and y portions of a {@code ParameticFunction}
 * must be supplied.
 * 
 * @since 1.0
 */
public class ParametricFunction {
    
    private final DerivableFunction x;
    private final DerivableFunction y;
    private final double MAX_INPUT;
    private double parameterTolerance;
    private final boolean goesForward;
    private final HashMap<Integer, CalculusFunction> dstFuncs;

    /**
     * Creates a new parametric function with x = h(t) specified by {@code x} and
     * y = k(t) specified by {@code y}. T can range from 0 to {@code MAX_INPUT}.
     * Overloads {@link #ParametricFunction(DerivableFunction, DerivableFunction,
     * double, boolean, double)} with direction assumed to be forward and a distance
     * tolerance of {@code 0.0001}. The distance tolerance can be later reset using
     * {@link #setDistanceTolerance(double)}. If a parameter outside the domain of
     * 0 to {@code MAX_INPUT} is supplied, position calculation will still work, but
     * conversion between parameter and distance along the function is likely to be
     * unstable.
     * 
     * @param x the function specifying the x of the parametric with respect to its parameter
     * @param y the function specifying the y of the parametric with respect to its parameter
     * @param MAX_INPUT the largest input parameter assumed to be a valid point on the function
     * @see Function
     * @since 1.0
     */
    public ParametricFunction(DerivableFunction x, DerivableFunction y, double MAX_INPUT) {
        this(x, y, MAX_INPUT, 0.0001);
    }
    
    /**
     * Creates a new parametric function with x = h(t) specified by {@code x} and
     * y = k(t) specified by {@code y}. T can range from 0 to {@code MAX_INPUT}.
     * Overloads {@link #ParametricFunction(DerivableFunction, DerivableFunction,
     * double, boolean, double)} with a distance tolerance of {@code 0.0001}. The
     * function is also given a direction that can be used by other code through
     * {@link #goesForward()}, but that is not used as a part of any internal
     * calculations. The distance tolerance can be later reset using
     * {@link #setDistanceTolerance(double)}. If a parameter outside the domain of
     * 0 to {@code MAX_INPUT} is supplied, position calculation will still work, but
     * conversion between parameter and distance along the function is likely to be
     * unstable.
     * 
     * @param x the function specifying the x of the parametric with respect to its parameter
     * @param y the function specifying the y of the parametric with respect to its parameter
     * @param MAX_INPUT the largest input parameter assumed to be a valid point on the function
     * @param forward whether or not the function travels forward
     * @see Function
     * @since 1.0
     */
    public ParametricFunction(DerivableFunction x, DerivableFunction y,
            double MAX_INPUT, boolean forward) {
        this(x, y, MAX_INPUT, forward, 0.0001);
    }
    
    
    /**
     * Creates a new parametric function with x = h(t) specified by {@code x} and
     * y = k(t) specified by {@code y}. T can range from 0 to {@code MAX_INPUT}.
     * Overloads {@link #ParametricFunction(DerivableFunction, DerivableFunction,
     * double, boolean, double)} If a parameter outside the domain of 0 to {@code MAX_INPUT}
     * is supplied, position calculation will still work, but conversion between
     * parameter and distance along the function is likely to be unstable. The distance
     * tolerance parameter is used as an input to a {@link BruteInverse} used in
     * {@link #getParameter(int)}. The direction is assumed to be forward.
     * 
     * @param x the function specifying the x of the parametric with respect to its parameter
     * @param y the function specifying the y of the parametric with respect to its parameter
     * @param MAX_INPUT the largest input parameter assumed to be a valid point on the function
     * @param parameterTolerance the allowed error of outputs of functions returned by
     *                          {@code getParameter(int)}
     * @see Function
     * @since 1.0
     */
    public ParametricFunction(DerivableFunction x, DerivableFunction y,
            double MAX_INPUT, double parameterTolerance) {
        this(x, y, MAX_INPUT, true, parameterTolerance);
    }
    
    /**
     * Creates a new parametric function with x = h(t) specified by {@code x} and
     * y = k(t) specified by {@code y}. T can range from 0 to {@code MAX_INPUT}.
     * Overloads {@link #ParametricFunction(DerivableFunction, DerivableFunction,
     * double, boolean, double)}. If a parameter outside the domain of 0 to {@code MAX_INPUT}
     * is supplied, position calculation will still work, but conversion between
     * parameter and distance along the function is likely to be unstable. The
     * function is also given a direction that can be used by other code through
     * {@link #goesForward()}, but that is not used as a part of any internal
     * calculations. The distance tolerance parameter is used as an input to a
     * {@link BruteInverse} used in {@link #getParameter(int)}.
     * 
     * @param x the function specifying the x of the parametric with respect to its parameter
     * @param y the function specifying the y of the parametric with respect to its parameter
     * @param MAX_INPUT the largest input parameter assumed to be a valid point on the function
     * @param forward whether or not the function travels forward
     * @param parameterTolerance the allowed error of outputs of functions returned by
     *                          {@code getParameter(int)}
     * @see Function
     * @since 1.0
     */
    public ParametricFunction(DerivableFunction x, DerivableFunction y, double MAX_INPUT,
            boolean forward, double parameterTolerance) {
        this.x = x;
        this.y = y;
        this.MAX_INPUT = MAX_INPUT;
        dstFuncs = new HashMap<>();
        goesForward = forward;
        this.parameterTolerance = parameterTolerance;
    }

    /**
     * Sets the input to a {@link BruteInverse} used in {@link #getParameter(int)}.
     * This is the allowed distance between the parameter returned by a parameter
     * getter function and the actual parameter required to go that distance along
     * the parametric function (subject to inaccuracies in functions returned by
     * {@link #getDistance(int)}).
     * 
     * @param parameterTolerance the new value for the {@code parameterTolerance}
     * @since 1.0
     */
    public void setDistanceTolerance(double parameterTolerance) {
        this.parameterTolerance = parameterTolerance;
    }
    
    /**
     * Returns the x and y outputs of the parametric function for the input parameter.
     * 
     * @param parameter the input, often a time measurement
     * @return the x-y output of the function at the input parameter
     * @since 1.0
     */
    public Point getXY(double parameter) {
        return new Point(x.get(parameter), y.get(parameter));
    }
    
    /**
     * Returns the x and y outputs of the parametric function for the input parameter,
     * as well as the slope. The slope is given as an {@link Angle}.
     * 
     * @param parameter the input, often a time measurement
     * @return the x-y-direction output of the function at the input parameter
     * @since 1.0
     */
    public Location getXYDir(double parameter) {
        double dy = y.derivative().get(parameter);
        double dx = x.derivative().get(parameter);
        double rad = Math.atan2(dy, dx);
        Angle dir = new Angle(rad, Angle.AngleUnit.RADIANS, Angle.AngleOrientation.UNIT_CIRCLE);
        double xOut = x.get(parameter);
        double yOut = y.get(parameter);
        return new Location(xOut, yOut, dir);
    }
    
    /**
     * Gives a function that converts parameters to distances along the parametric.
     * The function returned is estimated by a {@link BruteIntegral}, and the function
     * returned will be approximate.
     * 
     * @param INTEGRAL_NUM_SAMPLES accuracy specification as per {@link BruteIntegral}
     * @return a parameter to distance function for this parametric
     * @since 1.0
     */
    public synchronized CalculusFunction getDistance(int INTEGRAL_NUM_SAMPLES) {
        if (dstFuncs.containsKey(INTEGRAL_NUM_SAMPLES)) {
            return dstFuncs.get(INTEGRAL_NUM_SAMPLES);
        }
        CalculusFunction newFunc = new Calculify(new Composition.Derivable(
                new Sum.Derivable(
                        new Product(x.derivative(), x.derivative()),
                        new Product(y.derivative(), y.derivative())
                ),
                new PolynomialUpHalfDegree(1)
        ), INTEGRAL_NUM_SAMPLES, MAX_INPUT).integral();
        dstFuncs.put(INTEGRAL_NUM_SAMPLES, newFunc);
        return newFunc;
    }
    
    /**
     * Gives a function that converts distances to parameters along the parametric.
     * The function returned is estimated by a {@link BruteIntegral}, and the function
     * returned will be approximate. A {@link BruteInverse} is also required, and
     * its accuracy is specified either in the constructor or through the method
     * {@link #setDistanceTolerance(double)}.
     * 
     * @param INTEGRAL_NUM_SAMPLES accuracy specification as per {@link BruteIntegral}
     * @return a distance to parameter function for this parametric
     * @since 1.0
     */
    public DerivableInversibleFunction getParameter(int INTEGRAL_NUM_SAMPLES) {
        return new Inversiblify(getDistance(INTEGRAL_NUM_SAMPLES), parameterTolerance, MAX_INPUT).inverse();
    }

    /**
     * Gives the x component of this parametric, as specified in the constructor.
     * 
     * @return the x component of this parametric
     * @since 1.0
     */
    public DerivableFunction getX() {
        return x;
    }

    /**
     * Gives the y component of this parametric, as specified in the constructor.
     * 
     * @return the y component of this parametric
     * @since 1.0
     */
    public DerivableFunction getY() {
        return y;
    }

    /**
     * Gives the maximum input of this parametric, as specified in the constructor.
     * 
     * @return the maximum input of this parametric
     * @since 1.0
     */
    public double getMaxInput() {
        return MAX_INPUT;
    }
    
    /**
     * Prints a list of points on this parametric to {@code System.out}. One point
     * will be printed to each line, and a point will be printed for each parameter
     * between zero and the maximum input of this parametric, inclusive, at intervals
     * specified by {@code increment}.
     * 
     * @param increment the difference between parameters of two consecutively printed
     *                  points
     * @since 1.0
     */
    public void print(double increment) {
        for (double d = 0; d <= MAX_INPUT; d += increment) {
            Location loc = getXYDir(d);
            System.out.printf("(%f, %f)\n", loc.x, loc.y);
        }
    }

    /**
     * Gives whether this parametric goes forward, as specified in the constructor.
     * The only use of the {@code forward} constructor parameter is to be returned
     * here, so this function only concerns implementations that have chosen to use
     * it.
     * 
     * @return true if this parametric goes forward
     * @since 1.0
     */
    public boolean goesForward() {
        return goesForward;
    }
    
    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        String xStr = x.toString().replaceAll(Function.xVariableRegex(), "t");
        String yStr = y.toString().replaceAll(Function.xVariableRegex(), "t");
        return "\\left(" + xStr + ", " + yStr + "\\right)";
    }
    
    /**
     * 
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
        ParametricFunction par = (ParametricFunction) obj;
        return par.x.equals(x) && par.y.equals(y) && par.MAX_INPUT == MAX_INPUT && (par.goesForward == goesForward);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = 5;
        hash = 31 * hash + Objects.hashCode(this.x);
        hash = 31 * hash + Objects.hashCode(this.y);
        hash = 31 * hash + (int) (Double.doubleToLongBits(this.MAX_INPUT) ^ (Double.doubleToLongBits(this.MAX_INPUT) >>> 32));
        hash = 31 * hash + (this.goesForward ? 1 : 0);
        return hash;
    }
    
}