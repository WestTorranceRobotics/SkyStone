package org.westtorrancerobotics.lib.util;

public class MathUtils {
    
    public static boolean isZero(double d) {
        return Math.abs(d) < 1e-9;
    }
    
    public static double[] solveAugmentedMatrix(double[][] matrix) {
        for (int j = 0; j < matrix.length; j++) {
            for (int i = j; i < matrix.length && isZero(matrix[j][j]); i++) {
                swap(matrix, j, i);
            }
            multiply(matrix, j, -1/matrix[j][j]);
            for (int i = j+1; i < matrix.length; i++) {
                if (isZero(matrix[i][j])) {
                    continue;
                }
                multiply(matrix, i, 1/matrix[i][j]);
                add(matrix, j, i);
            }
            multiply(matrix, j, -1);
        }
        for (int j = matrix.length - 1; j >= 0; j--) {
            for (int i = 0; i < j; i++) {
                double sc = matrix[i][j];
                if (isZero(sc)) {
                    continue;
                }
                multiply(matrix, j, -sc);
                add(matrix, j, i);
                multiply(matrix, j, -1/sc);
            }
        }
        double[] solutions = new double[matrix.length];
        for (int i = 0; i < solutions.length; i++) {
            solutions[i] = matrix[i][matrix.length];
        }
        return solutions;
    }
    
    private static void swap(double[][] grid, int rowa, int rowb) {
        double[] row1 = new double[grid[0].length];
        System.arraycopy(grid[rowa], 0, row1, 0, row1.length);
        double[] row2 = new double[grid[0].length];
        System.arraycopy(grid[rowb], 0, row2, 0, row2.length);
        grid[rowb] = row1;
        grid[rowa] = row2;
    }
    
    private static void multiply(double[][] grid, int row, double scalar) {
        for (int i = 0; i < grid[row].length; i++) {
            grid[row][i] *= scalar;
        }
    }
    
    private static void add(double[][] grid, int rowa, int rowb) {
        for (int i = 0; i < grid[rowb].length; i++) {
            grid[rowb][i] += grid[rowa][i];
        }
    }
    
}
