package org.westtorrancerobotics.lib.functionmath.interfaces;

/**
 * A function whose indefinite integral and derivative can be evaluated to give a
 * function with the same characteristics. Effectively a {@link DerivableIntegrableFunction}
 * with more focused type outputs on its {@code derivative()} and {@code integral()}
 * methods.
 * 
 * @since 1.0
 */
public interface CalculusFunction extends DerivableIntegrableFunction<CalculusFunction> {
    
    /**
     * Gives a {@code CalculusFunction} that tells the slope of this function for
     * an x value equal to the input of the created function.
     * 
     * @return the derivative of this function
     * @since 1.0
     */
    @Override
    CalculusFunction derivative();
    
    /**
     * Gives a {@code CalculusFunction} that tells the area under this function
     * between the vertical lines at x-values of 0 and the input of the created function.
     * Area below the x-axis is counted negative.
     * 
     * @return the antiderivative of this function, with {@code C = 0}
     * @since 1.0
     */
    @Override
    CalculusFunction integral();
}
