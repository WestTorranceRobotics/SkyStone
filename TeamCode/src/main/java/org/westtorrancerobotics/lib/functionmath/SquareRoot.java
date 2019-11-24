package org.westtorrancerobotics.lib.functionmath;

/**
 * The square root function.
 * 
 * @since 1.0
 */
public class SquareRoot extends PolynomialUpHalfDegree {
    
    /**
     * Creates a square root function.
     * 
     * @since 1.0
     */
    public SquareRoot() {
        super(1);
    }

    /**
     * Gives the square root of the input. The output {@code y} will be such that
     * {@code y^2 = x}.
     * 
     * @param x the number of which to find the square root
     * @return the square root of the input
     */
    @Override
    public double get(double x) {
        return super.get(x);
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
        return SquareRoot.class.hashCode() + 1;
    }
    
}
