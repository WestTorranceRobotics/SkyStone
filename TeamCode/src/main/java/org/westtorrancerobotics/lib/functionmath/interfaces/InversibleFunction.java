package org.westtorrancerobotics.lib.functionmath.interfaces;

/**
 * A function whose inverse can be evaluated.
 * 
 * @since 1.0
 */
public interface InversibleFunction extends Function {
    /**
     * The reflection of this function over y = x. For any input of this function
     * with a definite output, the generated function will return the input when
     * supplied the definite output.
     * 
     * @return the inverse of this function
     * @since 1.0
     */
    InversibleFunction inverse();
}
