package org.westtorrancerobotics.lib.functionmath.trig;

import org.westtorrancerobotics.lib.functionmath.Composition;
import org.westtorrancerobotics.lib.functionmath.Constant;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;
import org.westtorrancerobotics.lib.functionmath.Difference;
import org.westtorrancerobotics.lib.functionmath.interfaces.InversibleFunction;
import org.westtorrancerobotics.lib.functionmath.Polynomial;
import org.westtorrancerobotics.lib.functionmath.Product;
import org.westtorrancerobotics.lib.functionmath.Quotient;
import org.westtorrancerobotics.lib.functionmath.SquareRoot;

/**
 * The inverse of the reciprocal of the cosine function (in radians).
 * 
 * @since 1.0
 */
public class Arcsecant implements DerivableFunction, InversibleFunction {

    /**
     * Creates an inverse of the reciprocal of the cosine function (in radians).
     * 
     * @since 1.0
     */
    public Arcsecant() {
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public DerivableFunction derivative() {
        return new Quotient(new Constant(1),
                new Product(new Composition.Derivable(
                        new Difference.Derivable(new Constant(1), new Quotient(
                                new Constant(1), new Polynomial(1, 0, 0)
                        )),
                        new SquareRoot()
                ),
                new Polynomial(1, 0, 0))
        );
    }

    /**
     * Gives the value y for which {@code cos(y) = 1/x}, with y measured in radians.
     * 
     * @param x the secant of the output angle
     * @return the output angle
     */
    @Override
    public double get(double x) {
        return Math.acos(1 / x);
    }

    /**
     * Returns the {@link Secant} function.
     * 
     * @return the reciprocal of the cosine function
     * @since 1.0
     */
    @Override
    public Secant inverse() {
        return new Secant();
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
        int hash = Arcsecant.class.hashCode() + 1;
        return hash;
    }
    
}
