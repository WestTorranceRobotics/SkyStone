package org.westtorrancerobotics.lib.functionmath;

import java.util.Objects;
import org.westtorrancerobotics.lib.functionmath.interfaces.CalculusFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;

/**
 * The addition function. Returns the sum of two functions.
 * 
 * @since 1.0
 */
public class Sum implements CalculusFunction {
    
    private final CalculusFunction a, b;

    /**
     * Creates a function that is the sum of the supplied functions.
     * 
     * @param a one addend
     * @param b another addend
     * @since 1.0
     */
    public Sum(CalculusFunction a, CalculusFunction b) {
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
        Sum s = new Sum(ad, bd);
        return s;
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public CalculusFunction integral() {
        return new Sum(a.integral(), b.integral());
    }

    /**
     * Returns the sum of the outputs of the two functions at the supplied
     * inputs. The value is {@code a.get(x) + b.get(x)}.
     * 
     * @param x the input for both input functions
     * @return the sum of the outputs of the contained functions
     */
    @Override
    public double get(double x) {
        return a.get(x) + b.get(x);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        return "\\left("+a+"+"+b+"\\right)";
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
        Sum sum = (Sum) obj;
        return sum.a.equals(a) && sum.b.equals(b);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = 3;
        hash = 79 * hash + Objects.hashCode(this.a);
        hash = 79 * hash + Objects.hashCode(this.b);
        return hash;
    }
    
    /**
     * The subtraction function. Returns the sum of two functions. Does not require
     * or return integrals.
     * 
     * @since 1.0
     */
    public static class Derivable implements DerivableFunction {
        
        private final DerivableFunction a, b;

        /**
         * Creates a function that is the sum of the supplied functions.
         * Does not require or return integrals.
         * 
         * @param a one addend
         * @param b another addend
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
            Derivable s = new Derivable(ad, bd);
            return s;
        }

        /**
         * Returns the sum of the outputs of the two functions at the supplied
         * inputs. The value is {@code a.get(x) + b.get(x)}.
         * 
         * @param x the input for both input functions
         * @return the sum of the outputs of the contained functions
         */
        @Override
        public double get(double x) {
            return a.get(x) + b.get(x);
        }
        
        /**
         * 
         * @since 1.0
         */
        @Override
        public String toString() {
            return "\\left("+a+"+"+b+"\\right)";
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
            Sum sum = (Sum) obj;
            return sum.a.equals(a) && sum.b.equals(b);
        }

        /**
         * 
         * @since 1.0
         */
        @Override
        public int hashCode() {
            int hash = 7;
            hash = 97 * hash + Objects.hashCode(this.a);
            hash = 97 * hash + Objects.hashCode(this.b);
            return hash;
        }
        
    }
    
}
