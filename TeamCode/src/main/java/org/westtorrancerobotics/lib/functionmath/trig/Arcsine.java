package org.westtorrancerobotics.lib.functionmath.trig;

import org.westtorrancerobotics.lib.functionmath.interfaces.InversibleFunction;

/**
 * The inverse of the sine function (in radians).
 * 
 * @since 1.0
 */
public class Arcsine implements InversibleFunction {

    /**
     * Creates an inverse of the sine function (in radians).
     * 
     * @since 1.0
     */
    public Arcsine() {
    }

    /**
     * Returns the {@link Sine} function.
     * 
     * @return the sine function
     * @since 1.0
     */
    @Override
    public Sine inverse() {
        return new Sine();
    }

    /**
     * Gives the value y for which {@code sin(y) = x}, with y measured in radians.
     * 
     * @param x the sine of the output angle
     * @return the output angle
     */
    @Override
    public double get(double x) {
        return Math.asin(x);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        return "\\arcsin\\left(x\\right)";
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
        int hash = Arcsine.class.hashCode() + 1;
        return hash;
    }
    
}
