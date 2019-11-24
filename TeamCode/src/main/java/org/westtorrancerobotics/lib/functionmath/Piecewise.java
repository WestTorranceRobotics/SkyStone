package org.westtorrancerobotics.lib.functionmath;

import java.util.Arrays;
import org.westtorrancerobotics.lib.functionmath.interfaces.CalculusFunction;
import org.westtorrancerobotics.lib.util.StringUtils;

/**
 * A piecewise function. Depending on the input, a variety of functions can be
 * performed on the input and then returned.
 * 
 * @since 1.0
 */
public class Piecewise implements CalculusFunction {
    
    private final CalculusFunction[] pieces;
    private final double[] startingPoints;
    private final double end;
    
    /**
     * Creates a piecewise function with the specified pieces, starting points,
     * and end point. If it is desired that the initial or final segment be unbounded,
     * {@code Double.POSITIVE_INFINITY} and {@code Double.NEGATIVE_INFINITY} should
     * be used. The number of starting points must be equal to the number of pieces,
     * and the starting points and end points must be finite and strictly increasing.
     * The starting and ending points create the domain restrictions for each portion
     * of the piecewise function. Each piece that is not the last ends where the
     * next one begins. If the input to the function is at a bounding point, the
     * higher (x-valued) segment will be used to determine the output.
     * 
     * @param pieces the individual portions of the function
     * @param startingPoints the low-end x-value for the domain of each piece
     * @param end the high-end x-value for the last piece
     * @throws IllegalArgumentException if the supplied starting points and end point
     *                                  are not finite and strictly increasing
     * @since 1.0
     */
    public Piecewise(CalculusFunction[] pieces, double[] startingPoints, double end) {
        if (pieces.length != startingPoints.length) {
            throw new IllegalArgumentException(
                    "Piecewise must have equal number of pieces and starting points."
            );
        }
        double last = Double.NaN;
        for (double start : startingPoints) {
            if (Double.isNaN(start)) {
                throw new IllegalArgumentException("NaN is an invalid starting point.");
            }
            if (Double.isNaN(last)) {
                last = start;
            } else {
                if (last >= start) {
                    throw new IllegalArgumentException("Starting points and end must be strictly increasing.");
                }
            }
        }
        if (Double.isNaN(Double.NaN)) {
            throw new IllegalArgumentException("NaN is an invalid ending point.");
        }
        if (last >= end) {
            throw new IllegalArgumentException("Starting points and end must be strictly increasing.");
        }
        this.pieces = pieces;
        this.startingPoints = startingPoints;
        this.end = end;
    }
    
    /**
     * Creates a piecewise function with the specified pieces and x-values of divisions
     * between pieces. Overloads {@link #Piecewise(CalculusFunction[], double[], double)}
     * with the parameter {@code end} combined into {@code startingPoints} as the
     * highest indexed value.
     * 
     * @param pieces the individual portions of the function
     * @param bounds the domain restrictions of the pieces
     * @throws IllegalArgumentException if the bounds are not finite and strictly increasing
     * @since 1.0
     */
    public Piecewise(CalculusFunction[] pieces, double... bounds) {
        this(pieces, removeLast(bounds), bounds[bounds.length - 1]);
    }
    
    // literally just cheating "call to this must be first statement in constructor"
    private static double[] removeLast(double[] original) {
        double[] output = new double[original.length - 1];
        System.arraycopy(original, 0, output, 0, output.length);
        return output;
    }
    
    /**
     * 
     * @since 1.0
     */
    @Override
    public Piecewise derivative() {
        CalculusFunction[] arr = new CalculusFunction[pieces.length];
        for (int i = 0; i < pieces.length; i++) {
            arr[i] = pieces[i].derivative();
        }
        return new Piecewise(arr, startingPoints, end);
    }
    
    /**
     * 
     * @throws ArithmeticException if the function is undefined at {@code x = 0}
     * @since 1.0
     */
    @Override
    public Piecewise integral() {
        if (startingPoints[0] > 0 || end < 0) {
            throw new ArithmeticException("No indefinite integral when undefined at zero.");
        }
        CalculusFunction[] newPieces = new CalculusFunction[pieces.length];
        for (int i = 0; i < pieces.length; i++) {
            newPieces[i] = new Sum(pieces[i].integral(), new Constant(integralConstant(i)));
        }
        return new Piecewise(newPieces, startingPoints, end);
    }

    /**
     * Returns the output of one of the pieces at the input. The piece selected
     * is the one for which the input supplied is greater than its starting point,
     * less than the next highest starting point, and less than the end point.
     * 
     * @param x the input of the piece in whose domain this parameter is
     * @return the output of the chosen piece
     * @throws ArithmeticException if the input is out of the domain of every piece
     * @since 1.0
     */
    @Override
    public double get(double x) {
        if (x < startingPoints[0]) {
            throw new ArithmeticException("Domain Error: Input too low.");
        }
        if (x > end) {
            throw new ArithmeticException("Domain Error: Input too high.");
        }
        for (int i = 0; i < startingPoints.length; i++) {
            if (x >= getBoundingPoint(i) && x < getBoundingPoint(i + 1)) {
                return pieces[i].get(x);
            }
        }
        return pieces[startingPoints.length - 1].get(x);
    }

    private double integralConstant(int pieceIndex) {
        int zeroContainer = -1;
        for (int i = 0; i < startingPoints.length; i++) {
            if (0 >= getBoundingPoint(i) && 0 <= getBoundingPoint(i + 1)) {
                zeroContainer = i;
                break;
            }
        }
        double count = 0;
        count -= pieces[zeroContainer].integral().get(0);
        if (pieceIndex > zeroContainer) {
            while (pieceIndex > zeroContainer) {
                double bound = getBoundingPoint(zeroContainer + 1);
                count -= pieces[zeroContainer + 1].integral().get(bound);
                count += pieces[zeroContainer].integral().get(bound);
                zeroContainer++;
                count -= pieces[zeroContainer].integral().get(0);
            }
        } else {
            while (pieceIndex < zeroContainer) {
                double bound = getBoundingPoint(zeroContainer);
                count -= pieces[zeroContainer - 1].integral().get(bound);
                count += pieces[zeroContainer].integral().get(bound);
                zeroContainer--;
                count -= pieces[zeroContainer].integral().get(0);
            }
        }
        return count;
    }
    
    private String getBoundingPointStr (int i) {
        if (i == startingPoints.length) {
            return StringUtils.formatDouble(end);
        }
        return StringUtils.formatDouble(startingPoints[i]);
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
            CalculusFunction seg = pieces[i];
            if (!Double.isFinite(getBoundingPoint(i)) && !Double.isFinite(getBoundingPoint(i + 1))) {
                strBuild.append(seg.toString()).append(",");
            } else if (!Double.isFinite(getBoundingPoint(i))) {
                strBuild.append("x<").append(getBoundingPointStr(i + 1)).append(":").append(seg.toString()).append(",");
            } else if (!Double.isFinite(getBoundingPoint(i + 1))) {
                strBuild.append(getBoundingPointStr(i)).append("<x:").append(seg.toString()).append(",");
            } else {
                strBuild.append(getBoundingPointStr(i)).append("<x<").append(getBoundingPointStr(i + 1))
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
        Piecewise pcw = (Piecewise) obj;
        return Arrays.equals(pcw.pieces, pieces) && Arrays.equals(pcw.startingPoints, startingPoints) && pcw.end == end;
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = 5;
        hash = 71 * hash + Arrays.deepHashCode(this.pieces);
        hash = 71 * hash + Arrays.hashCode(this.startingPoints);
        hash = 71 * hash + (int) (Double.doubleToLongBits(this.end) ^ (Double.doubleToLongBits(this.end) >>> 32));
        return hash;
    }
    
}
