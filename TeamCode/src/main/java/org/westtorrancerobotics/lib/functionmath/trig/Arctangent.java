package org.westtorrancerobotics.lib.functionmath.trig;

import org.westtorrancerobotics.lib.functionmath.Constant;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableInversibleFunction;
import org.westtorrancerobotics.lib.functionmath.Polynomial;
import org.westtorrancerobotics.lib.functionmath.Quotient;

/**
 * The inverse of the tangent function.
 * 
 * @since 1.0
 */
public class Arctangent implements DerivableInversibleFunction {

    /**
     * Creates an inverse of the tangent function (in radians).
     * 
     * @since 1.0
     */
    public Arctangent() {
    }

    /**
     * Gives the value y for which {@code tan(y) = x}, with y measured in radians.
     * 
     * @param x the tangent of the output angle
     * @return the output angle
     */
    @Override
    public double get(double x) {
        return Math.atan(x);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public DerivableFunction derivative() {
        return new Quotient(new Constant(1), new Polynomial(1, 0, 1));
    }

    /**
     * Returns the {@link Tangent} function.
     * 
     * @return the tangent function
     * @since 1.0
     */
    @Override
    public Tangent inverse() {
        return new Tangent();
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        return "\\cos^{-1}\\left(\\frac{1}{x}\\right)";
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
        int hash = Arctangent.class.hashCode() + 1;
        return hash;
    }
    
}