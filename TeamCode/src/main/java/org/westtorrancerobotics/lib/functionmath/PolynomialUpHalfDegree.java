package org.westtorrancerobotics.lib.functionmath;

import java.util.Arrays;
import org.westtorrancerobotics.lib.functionmath.interfaces.CalculusFunction;
import org.westtorrancerobotics.lib.util.StringUtils;

/**
 * A polynomial function with its exponents shifted, used for internal calculation.
 * Original intent was to create a square root function, with all exponents shifted
 * up {@code 0.5} as to make {@code y = 1} (which is {@code y = 1 * x ^ 0}) into
 * {@code y = sqrt(x)} (which is {@code y = 1 * x ^ 0.5}, but other applications
 * are possible.
 * 
 * @since 1.0
 */
class PolynomialUpHalfDegree implements CalculusFunction {
    
    private final double[] coefficients;
    private final double minPower;
    
    public PolynomialUpHalfDegree(double... coefficients) {
        this(coefficients, 0.5);
    }

    public PolynomialUpHalfDegree(double[] coefficients, double minPower) {
        this.minPower = minPower;
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
            if (coefficients.length - n >= 0) {
                System.arraycopy(coefficients, n, this.coefficients, 0, coefficients.length - n);
            }
        }
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public CalculusFunction derivative() {
        double[] arr = new double[coefficients.length];
        for (int i = 0; i < arr.length; i++) {
            double thisTermsPower = (arr.length - 1 - i + minPower);
            arr[i] = coefficients[i] * thisTermsPower;
        }
        return new PolynomialUpHalfDegree(arr, minPower - 1);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public CalculusFunction integral() {
        double[] arr = new double[coefficients.length];
        for (int i = 0; i < arr.length; i++) {
            double thisTermsPowerp1 = (arr.length - i + minPower);
            arr[i] = coefficients[i] / thisTermsPowerp1;
        }
        return new PolynomialUpHalfDegree(arr, minPower + 1);
    }
    
    @Override
    public double get(double x) {
        double n = 0;
        for (int i = degree(); i >= 0; i--) {
            n += Math.pow(x, minPower + i) * coefficients[coefficients.length - 1 - i];
        }
        return n;
    }
    
    private int degree() {
        return coefficients.length - 1;
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        String str = "\\left(";
        double deg = degree() + minPower;
        for (double coefficient : coefficients) {
            String xString = "";
            if (deg == 1) {
                xString = "x";
            } else if (deg != 0) {
                xString = "x^{" + StringUtils.formatDouble(deg) + "}";
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
            if (coefficient == 0) {
                fullString = "";
                str = str.substring(0, str.length() - 3);
            }
            str = str.concat(fullString);
            str += " + ";
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
        PolynomialUpHalfDegree puh = (PolynomialUpHalfDegree) obj;
        return Arrays.equals(puh.coefficients, coefficients) && puh.minPower == minPower;
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = 7;
        hash = 37 * hash + Arrays.hashCode(this.coefficients);
        hash = 37 * hash + (int) (Double.doubleToLongBits(this.minPower) ^ (Double.doubleToLongBits(this.minPower) >>> 32));
        return hash;
    }
}
