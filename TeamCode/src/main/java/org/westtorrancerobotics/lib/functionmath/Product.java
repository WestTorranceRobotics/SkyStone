package org.westtorrancerobotics.lib.functionmath;

import java.util.Objects;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;

/**
 * The multiplication function. Returns the product of two functions.
 * 
 * @since 1.0
 */
public class Product implements DerivableFunction {
    
    private final DerivableFunction a, b;

    /**
     * Creates a function that is the product of the supplied functions.
     * 
     * @param a one multiplicand
     * @param b another multiplicand
     * @since 1.0
     */
    public Product(DerivableFunction a, DerivableFunction b) {
        this.a = a;
        this.b = b;
    }
    
    /**
     * 
     * @since 1.0
     */
    @Override
    public DerivableFunction derivative() {
        DerivableFunction ad = a.derivative();
        DerivableFunction bd = b.derivative();
        Sum.Derivable s = new Sum.Derivable(new Product(ad, b), new Product(bd, a));
        return s;
    }

    /**
     * Returns the product of the outputs of the two functions at the supplied
     * inputs. The value is {@code a.get(x) * b.get(x)}.
     * 
     * @param x the input for both input functions
     * @return the product of the outputs of the contained functions
     */
    @Override
    public double get(double x) {
        return a.get(x) * b.get(x);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        return "\\left(" + a + "\\cdot " + b + "\\right)";
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
        Product prd = (Product) obj;
        return prd.a.equals(a) && prd.b.equals(b);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = 7;
        hash = 29 * hash + Objects.hashCode(this.a);
        hash = 29 * hash + Objects.hashCode(this.b);
        return hash;
    }
        
}
