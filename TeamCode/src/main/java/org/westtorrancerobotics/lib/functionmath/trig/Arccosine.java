package org.westtorrancerobotics.lib.functionmath.trig;

import org.westtorrancerobotics.lib.functionmath.interfaces.InversibleFunction;

/**
 * The inverse of the cosine function (in radians).
 * 
 * @since 1.0
 */
public class Arccosine implements InversibleFunction {

    /**
     * Creates an inverse of the cosine function (in radians).
     * 
     * @since 1.0
     */
    public Arccosine() {
    }

    /**
     * Returns the {@link Cosine} function.
     * 
     * @return the cosine function
     * @since 1.0
     */
    @Override
    public Cosine inverse() {
        return new Cosine();
    }

    /**
     * Gives the value y for which {@code cos(y) = x}, with y measured in radians.
     * 
     * @param x the cosine of the output angle
     * @return the output angle
     */
    @Override
    public double get(double x) {
        return Math.acos(x);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        return "\\arccos\\left(x\\right)";
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
        int hash = Arccosine.class.hashCode() + 1;
        return hash;
    }
    
}
