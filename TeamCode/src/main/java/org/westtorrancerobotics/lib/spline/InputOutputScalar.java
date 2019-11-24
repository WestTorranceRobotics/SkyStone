package org.westtorrancerobotics.lib.spline;

import org.westtorrancerobotics.lib.functionmath.Composition;
import org.westtorrancerobotics.lib.functionmath.Constant;
import org.westtorrancerobotics.lib.functionmath.Product;
import org.westtorrancerobotics.lib.functionmath.Quotient;
import org.westtorrancerobotics.lib.functionmath.casters.Calculify;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;

/**
 * Changes the respective units of the input and their correlation to the output.
 * Given a function that returns an output velocity with respect to time, this function
 * will turn in into a function that returns an output velocity with respect to position.
 * This only works for functions that have no roots in the specified domain.
 * 
 * @since 1.0
 */
public class InputOutputScalar implements DerivableFunction {
    
    private final DerivableFunction scaled;
    
    /**
     * Converts the function {@code original}'s input from time to position. Assumes
     * the output units of the first function were a velocity, and creates a function
     * with an output in velocity units. The domain of the function will be [0,
     * {@code maxInput}]. Due to evaluation of the integral of the reciprocal of the
     * given function, accuracy cannot be completely achieved, so the parameter
     * {@code INTEGERAL_NUM_SAMPLES} is provided, so that higher values give more
     * accuracy, and lower values give faster computation time.
     * 
     * @param original the function with time input and velocity output to change
     * @param INTEGRAL_NUM_SAMPLES specification of calculation accuracy for the function
     * @param maxInput specifier of the function's valid domain
     * @since 1.0
     */
    public InputOutputScalar (DerivableFunction original, int INTEGRAL_NUM_SAMPLES, double maxInput) {
        DerivableFunction integralReciprocalCalculator = new Calculify(
                new Quotient(new Constant(1), original), 10 * INTEGRAL_NUM_SAMPLES, maxInput).integral();
        double a = maxInput / integralReciprocalCalculator.get(maxInput);
        DerivableFunction integralReciprocal = new Calculify(
                new Quotient(new Constant(1), original), INTEGRAL_NUM_SAMPLES, maxInput).integral();
        this.scaled = new Composition.Derivable(new Product(new Constant(a), integralReciprocal), original);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public DerivableFunction derivative() {
        return scaled.derivative();
    }

    /**
     * 
     * @see #InputOutputScalar(DerivableFunction, int, double)
     * @since 1.0
     */
    @Override
    public double get(double x) {
        return scaled.get(x);
    }
}
