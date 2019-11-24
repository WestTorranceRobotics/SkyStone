package org.westtorrancerobotics.lib.functionmath.trig;

import org.westtorrancerobotics.lib.functionmath.Composition;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;
import org.westtorrancerobotics.lib.functionmath.Polynomial;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableInversibleFunction;

/**
 * The tangent function. Given an acute angle of a right triangle, in radians,
 * tangent yields the ratio of the length of the leg opposite the acute angle to
 * the length of the leg adjacent the acute angle.
 * 
 * @since 1.0
 */
public class Tangent implements DerivableInversibleFunction {
    
    /**
     * Creates a tangent function (in radians).
     * 
     * @since 1.0
     */
    public Tangent() {
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public DerivableFunction derivative() {
        return new Composition.Derivable(new Secant(), new Polynomial(1,0,0));
    }

    /**
     * Given an acute angle of a right triangle, in radians, the output is the
     * ratio of the length of the leg opposite the acute angle to the length of
     * the leg adjacent the acute angle.
     * 
     * @param x the measure of the acute angle, in radians
     * @return the ratio of the length of the leg opposite the acute angle to that
     *         of the leg adjacent the angle.
     * @since 1.0
     */
    @Override
    public double get(double x) {
        return Math.tan(x);
    }

    /**
     * The inverse of the tangent function.
     * 
     * @return the {@link Arctangent} function
     * @since 1.0
     */
    @Override
    public Arctangent inverse() {
        return new Arctangent();
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
        int hash = Tangent.class.hashCode() + 1;
        return hash;
    }
    
}
