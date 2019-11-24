package org.westtorrancerobotics.lib.functionmath.casters;

import java.util.Objects;
import org.westtorrancerobotics.lib.spline.geom.Point;
import org.westtorrancerobotics.lib.functionmath.interfaces.Function;
import org.westtorrancerobotics.lib.functionmath.Polynomial;
import org.westtorrancerobotics.lib.functionmath.PolynomialGenerator;

/**
 * Estimates the inverse of a strictly increasing or decreasing function. If a function's
 * inverse can be exactly evaluated, then it will be an instance of {@code InversibleFunction}
 * and its {@code .inverse()} method can be called. This function seeks to find an
 * input for which the output of the supplied function is very near to the input of
 * this function, and returns the found input. It is most fast and effective with
 * near linear functions.
 * 
 * @since 1.0
 */
public class BruteInverse implements Function {
    
    private final double ACCURACY;
    private final double MIN_INPUT;
    private final double MAX_INPUT;
    private final Function original;
    private boolean firstRun;
    private Polynomial firstLine;
    private Polynomial firstInvLine;
    
    /**
     * Creates the inverse of a function. The estimated result returned may differ
     * from the actual inverse by +/-{@code ACCURACY}. The parameters {@code MIN_INPUT}
     * and {@code MAX_INPUT} specify the domain over which the function is allowed
     * to be executed. The domain must restrict out any portions of the input function
     * where a constant value is returned for an extended group of inputs, or where
     * it switches from positive to negative slope for output to be predictable.
     * Additionally, execution will go faster if {@code MIN_INPUT} and {@code MAX_INPUT}
     * have values that more closely contain the outputs that will be desired of the
     * inverse.
     * 
     * @param original the function of which this is the inverse
     * @param ACCURACY the tolerance of error in the returned outputs
     * @param MIN_INPUT the lower domain restriction on the input function
     * @param MAX_INPUT the higher domain restriction on the input function
     * @throws IllegalArgumentException if {@code ACCURACY} is not positive or either
     *                                  domain restriction is indefinite
     * @since 1.0
     */
    public BruteInverse(Function original, double ACCURACY, double MIN_INPUT, double MAX_INPUT) {
        if (ACCURACY <= 0) {
            throw new IllegalArgumentException("Accuracy must be positive:" + ACCURACY);
        }
        if (!Double.isFinite(MIN_INPUT) || !Double.isFinite(MAX_INPUT)) {
            throw new IllegalArgumentException("Domain restrictions must be finite: ["
                    + MIN_INPUT + ", " + MAX_INPUT + "]");
        }
        this.ACCURACY = ACCURACY;
        this.MIN_INPUT = MIN_INPUT;
        this.MAX_INPUT = MAX_INPUT;
        this.original = original;
        this.firstRun = true;
    }

    /**
     * Returns the value of the inverse of the function at the specified y value.
     * 
     * @param y the output of the original function
     * @return the input of the original function to generate {@code y}
     * @throws IndexOutOfBoundsException if the outputs of the original function at
     *                                   the domain restrictions do not contain the
     *                                   supplied y value between them, inclusive.
     * @since 1.0
     */
    @Override
    public double get(double y) {
//        if (original instanceof InversibleFunction) {
//            System.out.println(original);
//            return ((InversibleFunction) original).inverse().get(y);
//        }
        Point guess1 = new Point(MIN_INPUT, original.get(MIN_INPUT));
        double guess1dst = Math.abs(guess1.y - y);
        if (guess1dst < ACCURACY) {
            return guess1.x;
        }
        Point guess2 = new Point(MAX_INPUT, original.get(MAX_INPUT));
        double guess2dst = Math.abs(guess2.y - y);
        if (guess2dst < ACCURACY) {
            return guess2.x;
        }
        if (!isBetween(guess1.y, y, guess2.y)) {
            throw new IndexOutOfBoundsException(guess1.y + ", " + y + ", " + guess2.y);
        }
        if (firstRun) {
            firstLine = PolynomialGenerator.generateLine(guess1, guess2);
            firstInvLine = firstLine.inverseLinear();
            firstRun = false;
        }
        double guess3x = firstInvLine.get(y);
        Point guess3 = new Point(guess3x, original.get(guess3x));
        double guess3dst = Math.abs(guess3.y - y);
        if (guess3dst < ACCURACY) {
            return guess3.x;
        }
        if (isBetween(guess1.y, y, guess3.y)) {
            guess2 = guess3;
        } else {
            guess1 = guess3;
        }
        while(true) {
            Polynomial line = PolynomialGenerator.generateLine(guess1, guess2);
            Polynomial invLine = line.inverseLinear();
            double guess4x = invLine.get(y);
            Point guess4 = new Point(guess4x, original.get(guess4x));
            double guess4dst = Math.abs(guess4.y - y);
            if (guess4dst < ACCURACY) {
                return guess4.x;
            }
            if (isBetween(guess1.y, y, guess4.y)) {
                guess2 = guess4;
            } else {
                guess1 = guess4;
            }
        }
    }
    
    private static boolean isBetween (double bound1, double x, double bound2) {
        if (bound1 <= x && x <= bound2) {
            return true;
        } else if (bound2 <= x && x <= bound1) {
            return true;
        }
        return false;
    }

    /**
     * Gives a string representation of the function through the original. The String
     * will be of the form {@code "Inverse(<function>)"}, where {@code <function>}
     * is replaced by the string representation of the original function.
     * <p>
     * THIS IMPLEMENTATION OF {@link Function#toString()} DOES NOT RETURN LaTeX CODE!
     * 
     * @return a string representation of the function
     * @since 1.0
     */
    @Override
    public String toString() {
        return "Inverse(" + original.toString() + ")";
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
        BruteInverse bri = (BruteInverse) obj;
        return bri.original.equals(original) && bri.ACCURACY == ACCURACY &&
                bri.MIN_INPUT == bri.MIN_INPUT && bri.MAX_INPUT == MAX_INPUT;
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = 3;
        hash = 89 * hash + (int) (Double.doubleToLongBits(this.ACCURACY) ^ (Double.doubleToLongBits(this.ACCURACY) >>> 32));
        hash = 89 * hash + (int) (Double.doubleToLongBits(this.MIN_INPUT) ^ (Double.doubleToLongBits(this.MIN_INPUT) >>> 32));
        hash = 89 * hash + (int) (Double.doubleToLongBits(this.MAX_INPUT) ^ (Double.doubleToLongBits(this.MAX_INPUT) >>> 32));
        hash = 89 * hash + Objects.hashCode(this.original);
        return hash;
    }
}
