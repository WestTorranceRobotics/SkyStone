package org.westtorrancerobotics.lib.functionmath.interfaces;

/**
 * A function whose indefinite integral can be evaluated.
 * 
 * @param <T> the type of the returned integral
 * @since 1.0
 */
public interface IntegrableFunction<T extends Function> extends Function {
    /**
     * Gives a function of the specified type that tells the area under this function
     * between the vertical lines at x-values of 0 and the input of the created function.
     * Area below the x-axis is counted negative. The type is specified as a type
     * parameter of this class.
     * 
     * @return the antiderivative of this function, with {@code C = 0}
     * @since 1.0
     */
    public T integral();
}
