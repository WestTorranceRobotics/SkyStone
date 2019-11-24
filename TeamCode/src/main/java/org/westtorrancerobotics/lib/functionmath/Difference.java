package org.westtorrancerobotics.lib.functionmath;

import java.util.Objects;
import org.westtorrancerobotics.lib.functionmath.interfaces.CalculusFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;

/**
 * The subtraction function. Returns the positive or negative distance between two
 * functions.
 * 
 * @since 1.0
 */
public class Difference implements CalculusFunction {
    
    private final CalculusFunction a, b;

    /**
     * Creates a function that is the difference of the supplied functions.
     * 
     * @param a the minuend
     * @param b the subtrahend
     * @since 1.0
     */
    public Difference(CalculusFunction a, CalculusFunction b) {
        this.a = a;
        this.b = b;
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public CalculusFunction derivative() {
        CalculusFunction ad = a.derivative();
        CalculusFunction bd = b.derivative();
        Difference d = new Difference(ad, bd);
        return d;
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public CalculusFunction integral() {
        return new Difference(a.integral(), b.integral());
    }

    /**
     * Returns the difference between the outputs of the two functions at the supplied
     * inputs. The value is {@code a.get(x) - b.get(x)}.
     * 
     * @param x the input for both input functions
     * @return the difference between the outputs of the contained functions
     */
    @Override
    public double get(double x) {
        return a.get(x) - b.get(x);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        return "\\left("+a+"-"+b+"\\right)";
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
        Difference dif = (Difference) obj;
        return dif.a.equals(a) && dif.b.equals(b);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = 5;
        hash = 41 * hash + Objects.hashCode(this.a);
        hash = 41 * hash + Objects.hashCode(this.b);
        return hash;
    }
    
    /**
     * The subtraction function. Returns the positive or negative distance between two
     * functions. Does not require or return integrals.
     * 
     * @since 1.0
     */
    public static class Derivable implements DerivableFunction {
        
        private final DerivableFunction a, b;

        /**
         * Creates a function that is the difference of the supplied functions.
         * Does not require or return integrals.
         * 
         * @param a the minuend
         * @param b the subtrahend
         * @since 1.0
         */
        public Derivable(DerivableFunction a, DerivableFunction b) {
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
            Derivable d = new Derivable(ad, bd);
            return d;
        }

        /**
         * Returns the difference between the outputs of the two functions at the
         * supplied inputs. The value is {@code a.get(x) - b.get(x)}.
         * 
         * @param x the input for both input functions
         * @return the difference between the outputs of the contained functions
         */
        @Override
        public double get(double x) {
            return a.get(x) - b.get(x);
        }
        
        /**
         * 
         * @since 1.0
         */
        @Override
        public String toString() {
            return "\\left("+a+"-"+b+"\\right)";
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
            Derivable der = (Derivable) obj;
            return der.a.equals(a) && der.b.equals(b);
        }

        /**
         * 
         * @since 1.0
         */
        @Override
        public int hashCode() {
            int hash = 5;
            hash = 13 * hash + Objects.hashCode(this.a);
            hash = 13 * hash + Objects.hashCode(this.b);
            return hash;
        }
        
    }
    
}