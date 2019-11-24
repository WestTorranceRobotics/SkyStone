package org.westtorrancerobotics.lib.functionmath.trig;

import org.westtorrancerobotics.lib.functionmath.Constant;
import org.westtorrancerobotics.lib.functionmath.Difference;
import org.westtorrancerobotics.lib.functionmath.Product;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableInversibleFunction;

/**
 * The reciprocal of the sine function. Given an acute angle of a right triangle,
 * in radians, cosecant yields the ratio of the length of the hypotenuse to the length
 * of the leg opposite the acute angle.
 * 
 * @since 1.0
 */
public class Cosecant implements DerivableInversibleFunction {
    
    /**
     * Creates a reciprocal of the sine function (in radians).
     * 
     * @since 1.0
     */
    public Cosecant() {
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public DerivableFunction derivative() {
        return new Difference.Derivable(new Constant(0), new Product(new Cotangent(), new Cosecant()));
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
        return 1 / Math.sin(x);
    }

    /**
     * The inverse of the reciprocal of the sine function.
     * 
     * @return the {@link Arccosecant} function
     * @since 1.0
     */
    @Override
    public Arccosecant inverse() {
        return new Arccosecant();
    }
    
    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        return "\\frac{1}{sin\\left(x\\right)}";
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
        int hash = Cosecant.class.hashCode() + 1;
        return hash;
    }
    
}
