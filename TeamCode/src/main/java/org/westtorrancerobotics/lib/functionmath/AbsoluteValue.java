package org.westtorrancerobotics.lib.functionmath;

import org.westtorrancerobotics.lib.functionmath.interfaces.CalculusFunction;

/**
 * The absolute value function. Gives the distance of a number from zero.
 * 
 * @since 1.0
 */
public class AbsoluteValue implements CalculusFunction {
    
    /** 
     * Creates a new absolute value function. Takes the absolute value of the identity
     * function.
     * 
     * @since 1.0
     */
    public AbsoluteValue() {
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public CalculusFunction derivative() {
        return new Piecewise(
                new CalculusFunction[]{
                        new Constant(-1),
                        new Constant(1)
                },
                new double[]{Double.NEGATIVE_INFINITY, 0}, Double.POSITIVE_INFINITY
        );
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public CalculusFunction integral() {
        return new Piecewise(
                new CalculusFunction[]{
                        new Polynomial(-0.5, 0, 0),
                        new Polynomial(0.5, 0, 0)
                },
                new double[]{Double.NEGATIVE_INFINITY, 0}, Double.POSITIVE_INFINITY
        );
    }

    /**
     * Gives the distance of the input from zero.
     * 
     * @param x the input number
     * @return the absolute value of the input
     */
    @Override
    public double get(double x) {
        return Math.abs(x);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        return "\\left|x\\right|";
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
        return AbsoluteValue.class.hashCode() + 1;
    }
    
}
