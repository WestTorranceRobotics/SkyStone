package org.westtorrancerobotics.lib.functionmath.casters;

import java.util.Objects;
import java.util.regex.Matcher;
import org.westtorrancerobotics.lib.functionmath.Constant;
import org.westtorrancerobotics.lib.functionmath.PolynomialGenerator;
import org.westtorrancerobotics.lib.functionmath.interfaces.CalculusFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.Function;
import org.westtorrancerobotics.lib.functionmath.interfaces.IntegrableFunction;

/**
 * Estimates the integral of a function whose exact integral cannot be exactly evaluated.
 * If a function's integral can be exactly evaluated, then it will be an instance
 * of {@code IntegrableFunction} and its {@code .integral()} method can be called.
 * This function uses an approximation of the original function using a set of cubic
 * functions, and takes the integral thereof to do its approximations. Its initialization
 * is lazy, and although the first call to this function can take hundreds of times
 * longer than would a call to {@code get} on an evaluated integral (or more, depending
 * on how much accuracy is requested), subsequent calls can be performed at extreme
 * speeds, with consistency but not perfect accuracy.
 * <p>
 * {@code BruteIntegral} is a subclass of {@code DerivableFunction} because its
 * derivative is just the original function supplied.
 * 
 * @since 1.0
 */
public class BruteIntegral implements DerivableFunction {
    
    private final int NUMBER_OF_SAMPLES;
    private final double MAX_INPUT;
    private final double tick;
    private final DerivableFunction input;
    private final CalculusFunction[] approx;
    private final double[] integralToApprox;
    private final double[] minusThis;
    private boolean approxDone;

    /**
     * Creates the integral of a function. Only assignment and preparation is done
     * in the constructor, the computationally intensive initialization of the
     * approximation function is performed on the first call of this function. The
     * accuracy of the function and its domain (specifically, [0, {@code MAX_INPUT}],
     * or [{@code MAX_INPUT}, 0] if {@code MAX_INPUT} is negative) are here supplied.
     * A value for input supplied outside the domain will not throw an exception,
     * but is not guaranteed to even remotely approximate the integral of the function.
     * 
     * @param input the function of which the antiderivative will be estimated
     * @param NUMBER_OF_SAMPLES the number of cubic pieces to use in approximation
     * @param MAX_INPUT the largest x-value within any of the cubic approximations,
     *                  by absolute value
     * @see BruteIntegral
     * @since 1.0
     */
    public BruteIntegral(DerivableFunction input, int NUMBER_OF_SAMPLES, double MAX_INPUT) {
        this.input = input;
        this.NUMBER_OF_SAMPLES = NUMBER_OF_SAMPLES;
        this.MAX_INPUT = MAX_INPUT;
        tick = MAX_INPUT / NUMBER_OF_SAMPLES;
        approx = new CalculusFunction[NUMBER_OF_SAMPLES + 1];
        integralToApprox = new double[NUMBER_OF_SAMPLES + 1];
        minusThis = new double[NUMBER_OF_SAMPLES + 1];
        approxDone = false;
    }

    private synchronized void initApprox() {
        if (!approxDone) {
            approxDone = true;
            double sum = 0;
            int i = 0;
            for (double str = 0; i < NUMBER_OF_SAMPLES; str += tick) {
                double mid1 = str+tick/3;
                double mid2 = str+tick*2/3;
                double end = str+tick;
                CalculusFunction pieceApprox = PolynomialGenerator.generateCubic(
                        str, input.get(str),
                        mid1, input.get(mid1),
                        mid2, input.get(mid2),
                        end, input.get(end));
                CalculusFunction pieceInt = pieceApprox.integral();
                sum += pieceInt.get(end) - pieceInt.get(str);
                integralToApprox[i + 1] = sum;
                approx[i] = pieceInt;
                minusThis[i] = pieceInt.get(str);
                i++;
            }
            integralToApprox[0] = 0;
            approx[NUMBER_OF_SAMPLES] = new Constant(0);
            minusThis[NUMBER_OF_SAMPLES] = 0;
        }
    }
    
    /**
     * Returns the approximate value of the antiderivative of the function. The
     * value at x = 0 will be 0. For values outside the specified domain of the function,
     * it is uncertain what the output will be. The first time this function is run,
     * any needed initialization will be run, and consequently, the function will
     * be much faster subsequently.
     * 
     * @param x the input of the integral
     * @return the output of the integral at the specified input
     * @see BruteIntegral
     * @see #BruteIntegral(DerivableFunction, int, double)
     * @since 1.0
     */
    @Override
    public double get(double x) {
        if (input instanceof IntegrableFunction) {
            return ((IntegrableFunction) input).integral().get(x);
        }
        initApprox();
        int i = (int) (x / tick);
        if (i > NUMBER_OF_SAMPLES) {
            i = NUMBER_OF_SAMPLES;
        }
        if (i < 0) {
            i = 0;
        }
        return integralToApprox[i] + approx[i].get(x) - minusThis[i];
    }
    
    /**
     * 
     * @since 1.0
     */
    @Override
    public DerivableFunction derivative() {
        return input;
    }
    
    private static final Object UOI_LOCK = new Object();
    private static volatile int usedObjIds = -1;
    private long objId = -1;
    
    private synchronized long getObjId() {
        if (objId == -1) {
            synchronized (UOI_LOCK) {
                usedObjIds++;
                objId = usedObjIds;
            }
        }
        return objId;
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        if (input instanceof IntegrableFunction) {
            return ((IntegrableFunction) input).integral().toString();
        }
        String myX = "x_{"+getObjId()+"}";
        String modifInputStr = input.toString().replaceAll(Function.xVariableRegex(), Matcher.quoteReplacement(myX));
        return "\\int_0^{x}" + modifInputStr + "d" + myX;
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
        BruteIntegral bri = (BruteIntegral) obj;
        return bri.input.equals(input) && bri.NUMBER_OF_SAMPLES == NUMBER_OF_SAMPLES && bri.MAX_INPUT == MAX_INPUT;
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = 7;
        hash = 37 * hash + this.NUMBER_OF_SAMPLES;
        hash = 37 * hash + (int) (Double.doubleToLongBits(this.MAX_INPUT) ^ (Double.doubleToLongBits(this.MAX_INPUT) >>> 32));
        hash = 37 * hash + Objects.hashCode(this.input);
        return hash;
    }
    
}