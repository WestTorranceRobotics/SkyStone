package org.westtorrancerobotics.lib.functionmath.casters;

import java.util.Objects;
import org.westtorrancerobotics.lib.functionmath.Composition;
import org.westtorrancerobotics.lib.functionmath.Constant;
import org.westtorrancerobotics.lib.functionmath.Quotient;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableInversibleFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.Function;
import org.westtorrancerobotics.lib.functionmath.interfaces.InversibleFunction;

/**
 * Makes a function that is an instance of {@code DerivableFunction} into an instance
 * of {@code DerivableInversibleFunction}. Uses the class {@link BruteInverse} to gain its
 * additional functionality.
 * 
 * @since 1.0
 */
public class Inversiblify implements DerivableInversibleFunction {
    
    private final DerivableFunction input;
    private final double ACCURACY;
    private final double MIN_INPUT;
    private final double MAX_INPUT;

    /**
     * Boxes the function {@code input}, leaving it with the same output and input
     * characteristics. The function gains the ability to be used as an
     * {@code InversibleFunction}, through use of estimation done in the class
     * {@link BruteInverse}. Overloads the constructor {@link
     * #Inversiblify(DerivableFunction, double, double, double)} with a minimum input
     * of zero.
     * 
     * @param input the function to box
     * @param ACCURACY the tolerance of error in the returned outputs to be used
     *                 when and if this function is inversed
     * @param MAX_INPUT the maximum input to be used when and if this function is
     *                  inversed
     * @see BruteInverse#BruteInverse(Function, double, double, double)
     * @since 1.0
     */
    public Inversiblify(DerivableFunction input, double ACCURACY, double MAX_INPUT) {
        this(input, ACCURACY, 0, MAX_INPUT);
    }

    /**
     * Boxes the function {@code input}, leaving it with the same output and input
     * characteristics. The function gains the ability to be used as an
     * {@code InversibleFunction}, through use of estimation done in the class
     * {@link BruteInverse}.
     * 
     * @param input the function to box
     * @param ACCURACY the tolerance of error in the returned outputs to be used
     *                 when and if this function is inversed
     * @param MIN_INPUT the minimum input to be used when and if this function is
     *                  inversed
     * @param MAX_INPUT the maximum input to be used when and if this function is
     *                  inversed
     * @see BruteInverse#BruteInverse(Function, double, double, double)
     * @since 1.0
     */
    public Inversiblify(DerivableFunction input, double ACCURACY, double MIN_INPUT, double MAX_INPUT) {
        this.input = input;
        this.ACCURACY = ACCURACY;
        this.MIN_INPUT = MIN_INPUT;
        this.MAX_INPUT = MAX_INPUT;
    }

    /**
     * Returns the output of the function supplied in initialization.
     * 
     * @param x the input for the original function
     * @return the output of the original function
     * @since 1.0
     */
    @Override
    public double get(double x) {
        return input.get(x);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public DerivableInversibleFunction inverse() {
        return new MyInverse(input, ACCURACY, MIN_INPUT, MAX_INPUT);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public DerivableFunction derivative() {
        return input.derivative();
    }

    /**
     * 
     * @see BruteInverse#toString()
     * @since 1.0
     */
    @Override
    public String toString() {
        return input.toString();
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
        Inversiblify inv = (Inversiblify) obj;
        return inv.input.equals(input) && inv.ACCURACY == ACCURACY && inv.MIN_INPUT == MIN_INPUT && inv.MAX_INPUT == MAX_INPUT;
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = 7;
        hash = 89 * hash + Objects.hashCode(this.input);
        hash = 89 * hash + (int) (Double.doubleToLongBits(this.ACCURACY) ^ (Double.doubleToLongBits(this.ACCURACY) >>> 32));
        hash = 89 * hash + (int) (Double.doubleToLongBits(this.MIN_INPUT) ^ (Double.doubleToLongBits(this.MIN_INPUT) >>> 32));
        hash = 89 * hash + (int) (Double.doubleToLongBits(this.MAX_INPUT) ^ (Double.doubleToLongBits(this.MAX_INPUT) >>> 32));
        return hash;
    }
    
    private class MyInverse extends BruteInverse implements DerivableInversibleFunction {
        
        private MyInverse(DerivableFunction input, double ACCURACY, double MIN_INPUT, double MAX_INPUT) {
            super(input, ACCURACY, MIN_INPUT, MAX_INPUT);
        }

        @Override
        public DerivableInversibleFunction inverse() {
            return Inversiblify.this;
        }
        
        @Override
        public DerivableFunction derivative() {
            return new Quotient(new Constant(1),
                    new Composition.Derivable(this, Inversiblify.this.derivative()));
        }
        
    }

    /**
    * Makes a function that is an instance of {@code Function} into an instance
    * of {@code InversibleFunction}. Uses the class {@link BruteInverse} to gain its
    * additional functionality.
    * 
    * @since 1.0
     */
    public static class NonDerivable implements InversibleFunction {
        
        private final Function input;
        private final double ACCURACY;
        private final double MIN_INPUT;
        private final double MAX_INPUT;

        /**
         * Boxes the function {@code input}, leaving it with the same output and input
         * characteristics. The function gains the ability to be used as an
         * {@code InversibleFunction}, through use of estimation done in the class
         * {@link BruteInverse}. Overloads the constructor {@link
         * #NonDerivable(Function, double, double, double)} with a minimum input
         * of zero.
         * 
         * @param input the function to box
         * @param ACCURACY the tolerance of error in the returned outputs to be used
         *                 when and if this function is inversed
         * @param MAX_INPUT the maximum input to be used when and if this function is
         *                  inversed
         * @see BruteInverse#BruteInverse(Function, double, double, double)
         * @since 1.0
         */
        public NonDerivable(Function input, double ACCURACY, double MAX_INPUT) {
            this.input = input;
            this.ACCURACY = ACCURACY;
            this.MIN_INPUT = 0;
            this.MAX_INPUT = MAX_INPUT;
        }

        /**
         * Boxes the function {@code input}, leaving it with the same output and input
         * characteristics. The function gains the ability to be used as an
         * {@code InversibleFunction}, through use of estimation done in the class
         * {@link BruteInverse}.
         * 
         * @param input the function to box
         * @param ACCURACY the tolerance of error in the returned outputs to be used
         *                 when and if this function is inversed
         * @param MIN_INPUT the minimum input to be used when and if this function is
         *                  inversed
         * @param MAX_INPUT the maximum input to be used when and if this function is
         *                  inversed
         * @see BruteInverse#BruteInverse(Function, double, double, double)
         * @since 1.0
         */
        public NonDerivable(Function input, double ACCURACY, double MIN_INPUT, double MAX_INPUT) {
            this.input = input;
            this.ACCURACY = ACCURACY;
            this.MIN_INPUT = MIN_INPUT;
            this.MAX_INPUT = MAX_INPUT;
        }

        /**
         * 
         * @since 1.0
         */
        @Override
        public InversibleFunction inverse() {
            return new MyInverse(input, ACCURACY, MIN_INPUT, MAX_INPUT);
        }

        /**
         * Returns the output of the function supplied in initialization.
         * 
         * @param x the input for the original function
         * @return the output of the original function
         * @since 1.0
         */
        @Override
        public double get(double x) {
            return input.get(x);
        }

        /**
         * 
         * @see BruteInverse#toString()
         * @since 1.0
         */
        @Override
        public String toString() {
            return input.toString();
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
            NonDerivable ndv = (NonDerivable) obj;
            return ndv.input.equals(input) && ndv.ACCURACY == ACCURACY && ndv.MAX_INPUT == MIN_INPUT && ndv.MAX_INPUT == MAX_INPUT;
        }

        /**
         * 
         * @since 1.0
         */
        @Override
        public int hashCode() {
            int hash = 5;
            hash = 43 * hash + Objects.hashCode(this.input);
            hash = 43 * hash + (int) (Double.doubleToLongBits(this.ACCURACY) ^ (Double.doubleToLongBits(this.ACCURACY) >>> 32));
            hash = 43 * hash + (int) (Double.doubleToLongBits(this.MIN_INPUT) ^ (Double.doubleToLongBits(this.MIN_INPUT) >>> 32));
            hash = 43 * hash + (int) (Double.doubleToLongBits(this.MAX_INPUT) ^ (Double.doubleToLongBits(this.MAX_INPUT) >>> 32));
            return hash;
        }
        
        private class MyInverse extends BruteInverse implements InversibleFunction {
        
            public MyInverse(Function input, double ACCURACY, double MIN_INPUT, double MAX_INPUT) {
                super(input, ACCURACY, MIN_INPUT, MAX_INPUT);
            }

            @Override
            public InversibleFunction inverse() {
                return NonDerivable.this;
            }
        }
        
    }
    
}