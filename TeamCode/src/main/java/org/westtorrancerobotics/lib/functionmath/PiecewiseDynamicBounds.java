package org.westtorrancerobotics.lib.functionmath;

import java.util.Arrays;
import java.util.Objects;
import java.util.function.DoubleUnaryOperator;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;

/**
 * A piecewise function. Depending on the output of the bounder function at the
 * input, a variety of functions can be performed on the input and then returned.
 * 
 * @since 1.0
 */
public class PiecewiseDynamicBounds implements DerivableFunction {
    
    private final DerivableFunction[] pieces;
    private final DoubleUnaryOperator bounder;
    private final double[] startingPoints;
    private final double end;
    
    /**
     * Creates a piecewise function with the specified pieces, starting points,
     * and end point. If it is desired that the initial or final segment be unbounded,
     * {@code Double.POSITIVE_INFINITY} and {@code Double.NEGATIVE_INFINITY} should
     * be used. The number of starting points must be equal to the number of pieces,
     * and the starting points and end points must be finite and strictly increasing.
     * The starting points, ending points, and bounder create the domain restrictions
     * for each portion of the piecewise function. All pieces except the last piece
     * end where the next one begins. If the output at the bounder at an input to
     * the function is a bounding point, the higher (x-valued) segment will be used
     * to determine the output.
     * 
     * @param pieces the individual portions of the function
     * @param bounder function used for determining which piece will be used for output
     * @param startingPoints the low-end x-value for the domain of each piece
     * @param end the high-end x-value for the last piece
     * @throws IllegalArgumentException if the supplied starting points and end point
     *                                  are not finite and strictly increasing
     * @since 1.0
     */
    public PiecewiseDynamicBounds(DerivableFunction[] pieces, DoubleUnaryOperator bounder,
            double[] startingPoints, double end) {
        if (pieces.length != startingPoints.length) {
            throw new IllegalArgumentException(
                    "Piecewise must have equal number of pieces and starting points."
            );
        }
        double last = Double.NaN;
        for (double start : startingPoints) {
            if (start == Double.NaN) {
                throw new IllegalArgumentException("NaN is an invalid starting point.");
            }
            if (last == Double.NaN) {
                last = start;
            } else {
                if (last >= start) {
                    throw new IllegalArgumentException("Starting points and end must be strictly increasing.");
                }
            }
        }
        if (end == Double.NaN) {
            throw new IllegalArgumentException("NaN is an invalid ending point.");
        }
        if (last >= end) {
            throw new IllegalArgumentException("Starting points and end must be strictly increasing.");
        }
        this.pieces = pieces;
        this.startingPoints = startingPoints;
        this.end = end;
        this.bounder = bounder;
    }
    
    /**
     * 
     * @since 1.0
     */
    @Override
    public PiecewiseDynamicBounds derivative() {
        DerivableFunction[] arr = new DerivableFunction[pieces.length];
        for (int i = 0; i < pieces.length; i++) {
            arr[i] = pieces[i].derivative();
        }
        return new PiecewiseDynamicBounds(arr, bounder, startingPoints, end);
    }

    /**
     * Returns the output of one of the pieces at the input. The piece selected
     * is the one for which the output of the bounder at the input supplied is
     * greater than its starting point, less than the next highest starting point,
     * and less than the end point.
     * 
     * @param x the input of the piece in whose domain this parameter is
     * @return the output of the chosen piece
     * @throws ArithmeticException if the input is out of the domain of every piece
     * @since 1.0
     */
    @Override
    public double get(double x) {
        double bounderX = bounder.applyAsDouble(x);
        if (bounderX < startingPoints[0]) {
            throw new ArithmeticException("Domain Error: Input too low.");
        }
        if (bounderX > end) {
            throw new ArithmeticException("Domain Error: Input too high.");
        }
        for (int i = 0; i < startingPoints.length; i++) {
            if (i + 1 == startingPoints.length) {
                return pieces[i].get(x);
            }
            if (bounderX >= startingPoints[i] && bounderX <= startingPoints[i + 1]) {
                return pieces[i].get(x);
            }
        }
        return 0; // this statement will never be called
    }
    
    private double getBoundingPoint (int i) {
        if (i == startingPoints.length) {
            return end;
        }
        return startingPoints[i];
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        StringBuilder strBuild = new StringBuilder("\\left\\{");
        for (int i = 0; i < pieces.length; i++) {
            DerivableFunction seg = pieces[i];
            if (!Double.isFinite(getBoundingPoint(i)) && !Double.isFinite(getBoundingPoint(i + 1))) {
                strBuild.append(seg.toString()).append(",");
            } else if (!Double.isFinite(getBoundingPoint(i))) {
                strBuild.append(bounder.toString()).append("<").append(getBoundingPoint(i + 1))
                        .append(":").append(seg.toString()).append(",");
            } else if (!Double.isFinite(getBoundingPoint(i + 1))) {
                strBuild.append(getBoundingPoint(i)).append("<").append(bounder.toString())
                        .append(":").append(seg.toString()).append(",");
            } else {
                strBuild.append(getBoundingPoint(i)).append("<").append(bounder.toString())
                        .append("<").append(getBoundingPoint(i + 1))
                        .append(":").append(seg.toString()).append(",");
            }
        }
        strBuild.deleteCharAt(strBuild.length() - 1);
        strBuild.append("\\right\\}");
        return strBuild.toString();
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
        PiecewiseDynamicBounds pdb = (PiecewiseDynamicBounds) obj;
        return pdb.bounder.equals(bounder) && Arrays.equals(pdb.pieces, pieces)
                && Arrays.equals(pdb.startingPoints, startingPoints) && pdb.end == end;
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = 7;
        hash = 47 * hash + Arrays.deepHashCode(this.pieces);
        hash = 47 * hash + Objects.hashCode(this.bounder);
        hash = 47 * hash + Arrays.hashCode(this.startingPoints);
        hash = 47 * hash + (int) (Double.doubleToLongBits(this.end) ^ (Double.doubleToLongBits(this.end) >>> 32));
        return hash;
    }
}
