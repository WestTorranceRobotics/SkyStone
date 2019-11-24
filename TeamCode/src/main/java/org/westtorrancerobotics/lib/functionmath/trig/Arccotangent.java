package org.westtorrancerobotics.lib.functionmath.trig;

import org.westtorrancerobotics.lib.functionmath.Constant;
import org.westtorrancerobotics.lib.functionmath.Polynomial;
import org.westtorrancerobotics.lib.functionmath.Quotient;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableInversibleFunction;

/**
 * The inverse of the reciprocal of the tangent function (in radians).
 * 
 * @since 1.0
 */
public class Arccotangent implements DerivableInversibleFunction {

    /**
     * Creates an inverse of the reciprocal of the tangent function (in radians).
     * 
     * @since 1.0
     */
    public Arccotangent() {
    }
    
    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        return "\\tan\\left(x\\right)";
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
        int hash = Arccotangent.class.hashCode() + 1;
        return hash;
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public DerivableFunction derivative() {
        return new Quotient(new Constant(-1),new Polynomial(1,0,1));
    }

    /**
     * Gives the value y for which {@code tan(y) = 1/x}, with y measured in radians.
     * 
     * @param x the secant of the output angle
     * @return the output angle
     */
    @Override
    public double get(double x) {
        return Math.atan(1 / x);
    }

    /**
     * Returns the {@link Cotangent} function.
     * 
     * @return the reciprocal of the tangent function
     * @since 1.0
     */
    @Override
    public Cotangent inverse() {
        return new Cotangent();
    }
    
}
