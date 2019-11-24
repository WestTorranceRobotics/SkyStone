package org.westtorrancerobotics.lib.functionmath.interfaces;

/**
 * A function whose derivative can be evaluated.
 * 
 * @since 1.0
 */
public interface DerivableFunction extends Function {
    /**
     * Gives a {@code DerivableFunction} that tells the slope of this function for
     * an x value equal to the input of the created function.
     * 
     * @return the derivative of this function
     * @since 1.0
     */
    DerivableFunction derivative();
}
