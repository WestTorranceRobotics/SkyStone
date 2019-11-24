package org.westtorrancerobotics.lib.functionmath.trig;

import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.InversibleFunction;
import org.westtorrancerobotics.lib.functionmath.Product;

/**
 * The reciprocal of the cosine function. Given an acute angle of a right triangle,
 * in radians, secant yields the ratio of the length of the hypotenuse to the length
 * of the leg adjacent the acute angle.
 * 
 * @since 1.0
 */
public class Secant implements DerivableFunction, InversibleFunction {

    /**
     * Creates a reciprocal of the cosine function (in radians).
     * 
     * @since 1.0
     */
    public Secant() {
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public DerivableFunction derivative() {
        return new Product(new Secant(), new Tangent());
    }

    /**
     * Given an acute angle of a right triangle, in radians, the output is the ratio
     * of the length of the hypotenuse to the length of the leg opposite the acute angle.
     * 
     * @param x the measure of the acute angle, in radians
     * @return the ratio of the length of the hypotenuse to that of the adjacent leg
     * @since 1.0
     */
    @Override
    public double get(double x) {
        return 1 / Math.cos(x);
    }

    /**
     * The inverse of the reciprocal of the cosine function.
     * 
     * @return the {@link Arcsecant} function
     * @since 1.0
     */
    @Override
    public Arcsecant inverse() {
        return new Arcsecant();
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        return "\\frac{1}{cos\\left(x\\right)}";
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
        int hash = Secant.class.hashCode() + 1;
        return hash;
    }
    
}