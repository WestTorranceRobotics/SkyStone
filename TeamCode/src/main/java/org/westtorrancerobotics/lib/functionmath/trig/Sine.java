package org.westtorrancerobotics.lib.functionmath.trig;

import org.westtorrancerobotics.lib.functionmath.Constant;
import org.westtorrancerobotics.lib.functionmath.Difference;
import org.westtorrancerobotics.lib.functionmath.interfaces.CalculusFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.ComplexFunction;

/**
 * The sine function. Given an acute angle of a right triangle, in radians, sine
 * yields the ratio of the length of the leg opposite the acute angle to the length
 * of the hypotenuse.
 * 
 * @since 1.0
 */
public class Sine implements ComplexFunction {

    /**
     * Creates a sine function (in radians).
     * 
     * @since 1.0
     */
    public Sine() {
    }

    /**
     * Given an acute angle of a right triangle, in radians, the output is the ratio
     * of the length of the leg opposite the acute angle to the length of the hypotenuse.
     * 
     * @param x the measure of the acute angle, in radians
     * @return the ratio of the length of the opposite leg to that of the hypotenuse
     * @since 1.0
     */
    @Override
    public double get(double x) {
        return Math.sin(x);
    }

    /**
     * Gives the cosine function.
     * 
     * @return the derivative of sine
     * @since 1.0
     */
    @Override
    public Cosine derivative() {
        return new Cosine();
    }

    /**
     * Gives additive inverse of the cosine function.
     * 
     * @return the antiderivative of sine
     * @since 1.0
     */
    @Override
    public CalculusFunction integral() {
        return new Difference(new Constant(0), new Cosine());
    }

    /**
     * The inverse of the sine function.
     * 
     * @return the {@link Arcsine} function
     * @since 1.0
     */
    @Override
    public Arcsine inverse() {
        return new Arcsine();
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        return "\\sin\\left(x\\right)";
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
        return obj.getClass().equals(getClass());
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = Sine.class.hashCode() + 1;
        return hash;
    }
    
}