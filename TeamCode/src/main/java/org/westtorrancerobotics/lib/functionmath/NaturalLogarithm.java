package org.westtorrancerobotics.lib.functionmath;

import java.util.Objects;
import org.westtorrancerobotics.lib.functionmath.interfaces.CalculusFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;

/**
 * The logarithm function, base e. Returns the natural logarithm of another function.
 * 
 * @since 1.0
 */
public class NaturalLogarithm implements DerivableFunction {
    
    private final CalculusFunction input;

    /**
     * Creates a new natural logarithm. Takes the {@code ln()} of the supplied input
     * function.
     * 
     * @param input the expression inside the parentheses of {@code ln()}
     */
    public NaturalLogarithm(CalculusFunction input) {
        this.input = input;
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public DerivableFunction derivative() {
        return new Quotient(input.derivative(), input);
    }

    /**
     * Takes the base e logarithm of the inner function at the domain {@code x}.
     * First executes the inner function, then takes the natural log of its output.
     * 
     * @param x the input for the inner function
     * @return the log of the output of the inner function
     */
    @Override
    public double get(double x) {
        return Math.log(input.get(x));
    }
    
    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        return "\\ln" + input.toString();
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
        if (!(obj.getClass().equals(getClass()))) {
            return false;
        }
        NaturalLogarithm lng = (NaturalLogarithm) obj;
        return lng.input.equals(input);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = 3;
        hash = 23 * hash + Objects.hashCode(this.input);
        return hash;
    }
    
}
