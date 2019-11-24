package org.westtorrancerobotics.lib.functionmath;

import java.util.Objects;
import java.util.function.DoubleUnaryOperator;
import java.util.regex.Matcher;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableInversibleFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.InversibleFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.DerivableFunction;
import org.westtorrancerobotics.lib.functionmath.interfaces.Function;

/**
 * Composes two functions. The {@link DoubleUnaryOperator#compose(DoubleUnaryOperator)}
 * method can be called on a function to give similar effects, but to keep mathematical
 * functions like derivation and inversion, as well as LaTeX string representation,
 * this class is provided also.
 * 
 * @since 1.0
 */
public class Composition implements DerivableFunction, InversibleFunction {

    private final DerivableInversibleFunction out;
    private final DerivableInversibleFunction in;

    /**
     * Creates a function that is the composition of its inputs. The first parameter
     * will be executed on the input first, and the second parameter will be executed
     * second (as if calling {@code out.compose(in)}).
     * 
     * @param in the inner function of the composition
     * @param out the outer function of the composition
     * @since 1.0
     */
    public Composition(DerivableInversibleFunction in, DerivableInversibleFunction out) {
        this.in = in;
        this.out = out;
    }

    /**
     * Returns the output of the second function when supplied an input equal to
     * the output of the first function at the supplied input. Specifically,
     * {@code out.get(in.get(x))} is returned.
     * 
     * @param x the input for the inner function
     * @return the output of the outer function
     * @since 1.0
     */
    @Override
    public double get(double x) {
        return out.get(in.get(x));
    }
    
    /**
     * 
     * @since 1.0
     */
    @Override
    public DerivableFunction derivative() {
        return new Product(new Derivable(in, out.derivative()), in.derivative());
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public Inversible inverse() {
        return new Inversible(out.inverse(), in.inverse());
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public String toString() {
        String outStr = out.toString();
        String inStr = in.toString();
        outStr = outStr.replaceAll(Function.xVariableRegex(), Matcher.quoteReplacement(inStr));
        return outStr;
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
        Composition com = (Composition) obj;
        return com.in.equals(in) && com.out.equals(out);
    }

    /**
     * 
     * @since 1.0
     */
    @Override
    public int hashCode() {
        int hash = 7;
        hash = 67 * hash + Objects.hashCode(this.out);
        hash = 67 * hash + Objects.hashCode(this.in);
        return hash;
    }
    
    /**
     * Composes two functions. The {@link DoubleUnaryOperator#compose(DoubleUnaryOperator)}
     * method can be called on a function to give similar effects, but to keep mathematical
     * functions like inversion and LaTeX string representation, this class is provided
     * also. Does not support or require derivatives.
     * 
     * @see Composition
     * @since 1.0
     */
    public static class Inversible implements InversibleFunction {
        
        private final InversibleFunction out;
        private final InversibleFunction in;

        /**
         * Creates a function that is the composition of its inputs. The first parameter
         * will be executed on the input first, and the second parameter will be executed
         * second (as if calling {@code out.compose(in)}). Does not support or require
         * derivatives.
         * 
         * @param in the inner function of the composition
         * @param out the outer function of the composition
         * @see Composition#Composition(DerivableInversibleFunction, DerivableInversibleFunction)
         * @since 1.0
         */
        public Inversible(InversibleFunction in, InversibleFunction out) {
            this.in = in;
            this.out = out;
        }

        /**
         * Returns the output of the second function when supplied an input equal to
         * the output of the first function at the supplied input. Specifically,
         * {@code out.get(in.get(x))} is returned.
         * 
         * @param x the input for the inner function
         * @return the output of the outer function
         * @see Composition#get(double)
         * @since 1.0
         */
        @Override
        public double get(double x) {
            return out.get(in.get(x));
        }

        /**
         * 
         * @since 1.0
         */
        @Override
        public Inversible inverse() {
            return new Inversible(out.inverse(), in.inverse());
        }

        /**
         * 
         * @since 1.0
         */
        @Override
        public String toString() {
            String outStr = out.toString();
            String inStr = in.toString();
            outStr = outStr.replaceAll(Function.xVariableRegex(), Matcher.quoteReplacement(inStr));
            return outStr;
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
            Inversible inv = (Inversible) obj;
            return inv.in.equals(in) && inv.out.equals(out);
        }

        /**
         * 
         * @since 1.0
         */
        @Override
        public int hashCode() {
            int hash = 5;
            hash = 79 * hash + Objects.hashCode(this.out);
            hash = 79 * hash + Objects.hashCode(this.in);
            return hash;
        }
    }
    
    /**
     * Composes two functions. The {@link DoubleUnaryOperator#compose(DoubleUnaryOperator)}
     * method can be called on a function to give similar effects, but to keep mathematical
     * functions like derivation and LaTeX string representation, this class is provided
     * also. Does not support or require inverses.
     * 
     * @see Composition
     * @since 1.0
     */
    public static class Derivable implements DerivableFunction {
        
        private final DerivableFunction out;
        private final DerivableFunction in;

        /**
         * Creates a function that is the composition of its inputs. The first parameter
         * will be executed on the input first, and the second parameter will be executed
         * second (as if calling {@code out.compose(in)}). Does not support or require
         * inverses.
         * 
         * @param in the inner function of the composition
         * @param out the outer function of the composition
         * @see Composition#Composition(DerivableInversibleFunction, DerivableInversibleFunction)
         * @since 1.0
         */
        public Derivable(DerivableFunction in, DerivableFunction out) {
            this.in = in;
            this.out = out;
        }

        /**
         * Returns the output of the second function when supplied an input equal to
         * the output of the first function at the supplied input. Specifically,
         * {@code out.get(in.get(x))} is returned.
         * 
         * @param x the input for the inner function
         * @return the output of the outer function
         * @see Composition#get(double)
         * @since 1.0
         */
        @Override
        public double get(double x) {
            return out.get(in.get(x));
        }

        /**
         * 
         * @since 1.0
         */
        @Override
        public DerivableFunction derivative() {
            DerivableFunction od = out.derivative();
            DerivableFunction id = in.derivative();
            return new Product(new Derivable(in, od), id);
        }

        /**
         * 
         * @since 1.0
         */
        @Override
        public String toString() {
            String outStr = out.toString();
            String inStr = in.toString();
            outStr = outStr.replaceAll(Function.xVariableRegex(), Matcher.quoteReplacement(inStr));
            return outStr;
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
            return der.in.equals(in) && der.out.equals(out);
        }

        /**
         * 
         * @since 1.0
         */
        @Override
        public int hashCode() {
            int hash = 3;
            hash = 53 * hash + Objects.hashCode(this.out);
            hash = 53 * hash + Objects.hashCode(this.in);
            return hash;
        }
    }   
}