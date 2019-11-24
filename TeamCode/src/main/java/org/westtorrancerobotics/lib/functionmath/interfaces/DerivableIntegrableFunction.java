package org.westtorrancerobotics.lib.functionmath.interfaces;

/**
 * A function whose indefinite integral and derivative can be evaluated.
 * 
 * @param <T> the type of the returned integral
 * @since 1.0
 */
public interface DerivableIntegrableFunction<T extends Function> extends
        DerivableFunction, IntegrableFunction<T> {
    
}
