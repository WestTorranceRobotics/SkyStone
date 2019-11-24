package org.westtorrancerobotics.lib.functionmath.trig;

import org.westtorrancerobotics.lib.functionmath.Composition;
import org.westtorrancerobotics.lib.functionmath.Constant;
import org.westtorrancerobotics.lib.functionmath.Difference;
import org.westtorrancerobotics.lib.functionmath.Polynomial;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableInversibleFunction;

/**
 * The reciprocal of the tangent function. Given an acute angle of a right triangle,
 * in radians, cotangent yields the ratio of the length of the leg adjacent the acute
 * angle to the length of the leg opposite the acute angle.
 * 
 * @since 1.0
 */
public class Cotangent implements DerivableInversibleFunction {
    
    /**
     * Creates a reciprocal of the tangent function (in radians).
     * 
     * @since 1.0
     */
    public Cotangent() {
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public DerivableFunction derivative() {
        return new Difference.Derivable(new Constant(0),
                new Composition.Derivable(new Cosecant(), new Polynomial(1, 0, 0))
        );
    }

    /**
     * Given an acute angle of a right triangle, in radians, the output is the
     * ratio of the length of the leg adjacent the acute angle to the length of
     * the leg opposite the acute angle.
     * 
     * @param x the measure of the acute angle, in radians
     * @return the ratio of the length of the leg adjacent the acute angle to that
     *         of the leg opposite the angle.
     * @since 1.0
     */
    @Override
    public double get(double x) {
        return 1 / Math.tan(x);
    }

    /**
     * The inverse of the reciprocal of the tangent function.
     * 
     * @return the {@link Arccotangent} function
     * @since 1.0
     */
    @Override
    public Arccotangent inverse() {
        return new Arccotangent();
    }
    
    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        return "\\frac{1}{tan\\left(x\\right)}";
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
        int hash = Cotangent.class.hashCode() + 1;
        return hash;
    }
    
}
