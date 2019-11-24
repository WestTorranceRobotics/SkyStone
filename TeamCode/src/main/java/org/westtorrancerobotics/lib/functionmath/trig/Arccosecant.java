package org.westtorrancerobotics.lib.functionmath.trig;

import org.westtorrancerobotics.lib.functionmath.Composition;
import org.westtorrancerobotics.lib.functionmath.Constant;
import org.westtorrancerobotics.lib.functionmath.Difference;
import org.westtorrancerobotics.lib.functionmath.Polynomial;
import org.westtorrancerobotics.lib.functionmath.Product;
import org.westtorrancerobotics.lib.functionmath.Quotient;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableInversibleFunction;

/**
 * The inverse of the reciprocal of the cosine function (in radians).
 * 
 * @since 1.0
 */
public class Arccosecant implements DerivableInversibleFunction {

    /**
     * Creates an inverse of the reciprocal of the cosine function (in radians).
     * 
     * @since 1.0
     */
    public Arccosecant() {
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public DerivableFunction derivative() {
        return new Quotient(new Constant(-1),
                new Product(new Polynomial(1,0,0),
                        new Composition.Derivable(new Difference.Derivable(
                                new Constant(1),
                                new Quotient(new Constant(1), new Polynomial(1,0,0))
                        ), new Polynomial(1))
                )
        );
    }

    /**
     * Gives the value y for which {@code sin(y) = 1/x}, with y measured in radians.
     * 
     * @param x the reciprocal of the sine of the output angle
     * @return the output angle
     */
    @Override
    public double get(double x) {
        return Math.asin(1 / x);
    }

    /**
     * Returns the {@link Cosecant} function.
     * 
     * @return the reciprocal of the sine function
     * @since 1.0
     */
    @Override
    public Cosecant inverse() {
        return new Cosecant();
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
        int hash = Arccosecant.class.hashCode() + 1;
        return hash;
    }
    
}
