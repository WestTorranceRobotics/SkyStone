package org.westtorrancerobotics.lib.functionmath;

import java.util.Arrays;
import org.westtorrancerobotics.lib.functionmath.interfaces.CalculusFunction;
import org.westtorrancerobotics.lib.util.StringUtils;

/**
 * A polynomial function.
 * 
 * @since 1.0
 */
public class Polynomial implements CalculusFunction {
    
    private final double[] coefficients;

    /**
     * Creates a polynomial with the specified coefficients, in order of decreasing
     * degree. The last number in {@code coefficients} will be the constant term.
     * Non-leading zero terms cannot be omitted. For example, {@code f(x) = 2x + 1}
     * is given by {@code new Polynomial(2, 1)}, and {@code f(x) = x^2} is given
     * by {@code new Polynomial(1, 0, 0)}.
     * 
     * @param coefficients the coefficients of the terms summed to make this polynomial
     * @since 1.0
     */
    public Polynomial(double... coefficients) {
        if (coefficients.length == 0) {
            this.coefficients = new double[]{0};
            return;
        }
        if (coefficients[0] != 0) {
            this.coefficients = coefficients;
        }
        else {
            int n = 0;
            try {
                while (coefficients[n] == 0) {
                    n++;
                }
            }
            catch (ArrayIndexOutOfBoundsException e) {
                this.coefficients = new double[]{0};
                return;
            }
            this.coefficients = new double[coefficients.length - n];
            for (int i = n; i < coefficients.length; i++) {
                this.coefficients[i - n] = coefficients[i];
            }
        }
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public Polynomial derivative() {
        double[] arr = new double[coefficients.length - 1];
        for (int i = 0; i < arr.length; i++) {
            arr[i] = coefficients[i] * (arr.length - i);
        }
        return new Polynomial(arr);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public Polynomial integral() {
        double[] arr = new double[coefficients.length + 1];
        for (int i = 0; i < coefficients.length; i++) {
            arr[i] = coefficients[i] / (coefficients.length - i);
        }
        return new Polynomial(arr);
    }

    /**
     * Gives the value of the polynomial for the input supplied.
     * 
     * @param x the x input to the polynomial
     * @return the value of the polynomial
     * @since 1.0
     */
    @Override
    public double get(double x) {
        double n = 0;
        double m = 1;
        for (int i = degree(); i >= 0; i--) {
            n += m * coefficients[i];
            m *= x;
        }
        return n;
    }
    
    /**
     * Returns the degree of the polynomial. The constant polynomial with value zero
     * will be shown as zeroth degree, even though it is in fact without a degree.
     * The number of coefficients, leading zeros ignored, but trailing zeros included,
     * is one greater than a polynomial's degree.
     * 
     * @return the degree of this function
     * @since 1.0
     */
    public int degree() {
        return coefficients.length - 1;
    }
    
    /**
     * Returns the inverse of this polynomial, if it is of degree one. If it is not a line,
     * or if its slope is zero, an {@code IllegalArgumentException} is thrown.
     * 
     * @return the inverse of this polynomial
     * @throws IllegalArgumentException if this polynomial is not of the first degree
     * @since 1.0
     */
    public Polynomial inverseLinear() {
        if (degree() != 1) {
            throw new IllegalArgumentException("Function not linear.");
        }
        double m = coefficients[0];
        double b = coefficients[1];
        double invm = 1 / m;
        return new Polynomial(invm, invm * b * -1);
    }

    /**
     * Returns the values for which this polynomial has the specified y-values, if this
     * polynomial is a parabola. Uses the quadratic formula. If there are no real
     * values such that this polynomial has the specified y-values, or if this polynomial
     * is not a parabola, exceptions are thrown. If there is only one unique such value,
     * it will be returned twice.
     * 
     * @param targY the output for which inputs are to be found
     * @return an array of length 2 containing the two inputs for which the specified
     *         output will be returned from this function
     * @throws ArithmeticException if there are no real inputs that give the specified output
     * @throws IllegalArgumentException if this function is not quadratic
     */
    public double[] inverseParabola (double targY) {
        if (degree() != 2) {
            throw new IllegalArgumentException("Function not quadratic:"+this);
        }
        double a = coefficients[0];
        double b = coefficients[1];
        double c = coefficients[2];
        double sqrt = Math.sqrt(b*b-4*a*(c-targY));
        if (!Double.isFinite(sqrt)) {
            sqrt=0;
            throw new ArithmeticException("Math failure:"+(b*b-4*a*(c-targY)));
        }
        return new double[]{(-b+sqrt)/(2*a), (-b-sqrt)/(2*a)};
    }
    
    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        String str = "\\left(";
        int deg = degree();
        if (deg == 0 && coefficients[0] == 0) {
            return "0";
        }
        for (double coefficient : coefficients) {
            String xString = "";
            switch (deg) {
                case 0:
                    break;
                case 1:
                    xString = "x";
                    break;
                default:
                    xString = "x^{" + deg + "}";
                    break;
            }
            String coString = StringUtils.formatDouble(coefficient);
            String fullString = coString + xString;
            if (coefficient == 1) {
                if (xString.equals("")) {
                    fullString = "1";
                } else {
                    fullString = xString;
                }
            }
            if (coefficient != 0) {
                str = str + fullString + " + ";
            }
            deg--;
        }
        return str.substring(0, str.length() - 3) + "\\right)";
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
        Polynomial ply = (Polynomial) obj;
        return Arrays.equals(ply.coefficients, coefficients);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = 3;
        hash = 47 * hash + Arrays.hashCode(this.coefficients);
        return hash;
    }
}
