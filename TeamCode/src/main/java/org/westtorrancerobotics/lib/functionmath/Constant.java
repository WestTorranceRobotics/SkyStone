package org.westtorrancerobotics.lib.functionmath;

import org.westtorrancerobotics.lib.functionmath.interfaces.CalculusFunction;
import org.westtorrancerobotics.lib.util.StringUtils;

/**
 * The constant function. For any value of c, this class can give a function {@code
 * f(x) = c} that will always output c.
 * 
 * @since 1.0
 */
public class Constant implements CalculusFunction {

    private final double value;
    
    /**
     * Creates a new constant function {@code f(x) = c}.
     * 
     * @param value the constant {@code c} that will always be returned
     */
    public Constant(double value) {
        this.value = value;
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public CalculusFunction derivative() {
        return new Constant(0);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public Polynomial integral() {
        return new Polynomial(value, 0);
    }

    /**
     * Returns the value supplied in the constructor.
     * 
     * @param x the input, which is ignored
     * @return the constant value of this function
     */
    @Override
    public double get(double x) {
        return value;
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        return StringUtils.formatDouble(value);
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
        Constant con = (Constant) obj;
        return con.value == value;
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = 7;
        hash = 29 * hash + (int) (Double.doubleToLongBits(this.value) ^ (Double.doubleToLongBits(this.value) >>> 32));
        return hash;
    }
    
}
