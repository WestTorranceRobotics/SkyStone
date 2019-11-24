package org.westtorrancerobotics.lib.functionmath;

import java.util.Objects;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;

/**
 * The division function. Returns the quotient of two functions.
 * 
 * @since 1.0
 */
public class Quotient implements DerivableFunction {
    
    private final DerivableFunction a;
    private final DerivableFunction b;

    /**
     * Creates a function that is the quotient of the supplied functions.
     * 
     * @param a the dividend
     * @param b the divisor
     * @since 1.0
     */
    public Quotient(DerivableFunction a, DerivableFunction b) {
        this.a = a;
        this.b = b;
    }
    
    /**
     * 
     * @since 1.0
     */
    @Override
    public DerivableFunction derivative() {
        return new Sum.Derivable(
                new Quotient(
                        new Product(new Product(a, b.derivative()), new Constant(-1)),
                        new Product(b, b)
                ),
                new Quotient(a.derivative(), b)
        );
    }

    /**
     * Returns the quotient of the outputs of the two functions at the supplied
     * inputs. The value is {@code a.get(x) / b.get(x)}.
     * 
     * @param x the input for both input functions
     * @return the quotient of the outputs of the contained functions
     */
    @Override
    public double get(double x) {
        return a.get(x) / b.get(x);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        return "\\frac{"+a+"}{"+b+"}";
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
        Quotient quo = (Quotient) obj;
        return quo.a.equals(a) && quo.b.equals(b);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = 7;
        hash = 37 * hash + Objects.hashCode(this.a);
        hash = 37 * hash + Objects.hashCode(this.b);
        return hash;
    }
    
}
