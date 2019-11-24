package org.westtorrancerobotics.lib.functionmath.trig;

import org.westtorrancerobotics.lib.functionmath.Constant;
import org.westtorrancerobotics.lib.functionmath.Difference;
import org.westtorrancerobotics.lib.functionmath.interfaces.CalculusFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.ComplexFunction;

/**
 * The cosine function. Given an acute angle of a right triangle, in radians, cosine
 * yields the ratio of the length of the leg adjacent the acute angle to the length
 * of the hypotenuse.
 * 
 * @since 1.0
 */
public class Cosine implements ComplexFunction {

    /**
     * Creates a cosine function (in radians).
     * 
     * @since 1.0
     */
    public Cosine() {
    }

    /**
     * Given an acute angle of a right triangle, in radians, the output is the ratio
     * of the length of the leg adjacent the acute angle to the length of the hypotenuse.
     * 
     * @param x the measure of the acute angle, in radians
     * @return the ratio of the length of the adjacent leg to that of the hypotenuse
     * @since 1.0
     */
    @Override
    public double get(double x) {
        return Math.cos(x);
    }

    /**
     * Gives additive inverse of the sine function.
     * 
     * @return the derivative of cosine
     * @since 1.0
     */
    @Override
    public CalculusFunction derivative() {
        return new Difference(new Constant(0), new Sine());
    }

    /**
     * Gives the sine function.
     * 
     * @return the antiderivative of cosine
     * @since 1.0
     */
    @Override
    public Sine integral() {
        return new Sine();
    }

    /**
     * The inverse of the cosine function.
     * 
     * @return the {@link Arccosine} function
     * @since 1.0
     */
    @Override
    public Arccosine inverse() {
        return new Arccosine();
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        return "\\cos\\left(x\\right)";
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
        int hash = Cosine.class.hashCode() + 1;
        return hash;
    }
    
}