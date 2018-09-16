package org.firstinspires.ftc.teamcode;


/**
 * This class contains static utility methods for dealing with matrices of
 * {@code double} values.
 *
 * @author Rodion "rodde" Efremov
 * @version 1.6
 */
public class MatrixUtils {

    private static final int DOUBLE_FORMAT_STRINGS = 0;
    private static final int STRING_FORMAT_STRINGS = 1;

    /**
     * The default separator length. A separator is a string of spaces used to
     * separate the values in the adjacent columns on the same row.
     */
    private static final int DEFAULT_SEPARATOR_LENGTH = 2;

    /**
     * Returns the string representation of the input matrix. The amount of
     * decimals in the output equals to the longest decimal part present in
     * {@code matrix}.
     *
     * @param matrix the matrix whose string representation to compute.
     * @return a string representing the input matrix.
     */
    public static String toString(final double[][] matrix) {
        final int lengthOfLongestDecimalPart =
                findLengthOfLongestDecimalPart(matrix);

        return toString(matrix,
                lengthOfLongestDecimalPart,
                DEFAULT_SEPARATOR_LENGTH);
    }

    /**
     * Returns the string representation of the input matrix. Each element in
     * the output has exactly {@code decimalPartLength} decimals and the length
     * of the separator is equal to {@code separatorLength} spaces.
     *
     * @param matrix            the input matrix.
     * @param decimalPartLength the amount of decimals to print.
     * @param separatorLength   the length of the separator.
     * @return a string representing the input matrix.
     */
    public static String toString(final double[][] matrix,
                                  int decimalPartLength,
                                  int separatorLength) {
        if (matrix == null) {
            return "null";
        }

        final int maximumRowLength = findMaximumRowLength(matrix);

        if (maximumRowLength == 0) {
            // Considered to be an empty matrix.
            return "";
        }

        // Make sure that the requested decimal part length is nonnegative.
        decimalPartLength = Math.max(decimalPartLength, 0);

        // Make sure that the separator length is nonnegative.
        separatorLength = Math.max(separatorLength, 0);

        final String[][] formatStringData =
                createColumnFormatStrings(matrix,
                        maximumRowLength,
                        decimalPartLength);

        final String separatorString =
                getColumnSeparatorString(separatorLength);

        final String missingElementString =
                getMissingElementString(decimalPartLength);

        final StringBuilder sb = new StringBuilder();

        rowToString(sb,
                matrix[0],
                formatStringData,
                separatorString,
                missingElementString);

        for (int y = 1; y < matrix.length; ++y) {
            sb.append('\n');
            rowToString(sb,
                    matrix[y],
                    formatStringData,
                    separatorString,
                    missingElementString);
        }

        return sb.toString();
    }

    /**
     * Returns the length of the longest row in the input matrix.
     *
     * @param matrix the matrix to process.
     * @return the length of the longest row in the matrix.
     */
    private static int findMaximumRowLength(final double[][] matrix) {
        int maximumRowLength = 0;

        for (final double[] row : matrix) {
            if (row != null) {
                maximumRowLength = Math.max(maximumRowLength, row.length);
            }
        }

        return maximumRowLength;
    }

    /**
     * Returns the length of the longest decimal number sequence present in the
     * input matrix.
     *
     * @param matrix the matrix in which perform the search.
     * @return the length in characters of the longest decimal part present in
     *         the matrix.
     */
    private static int findLengthOfLongestDecimalPart(final double[][] matrix) {
        int lengthOfLongestDecimalPart = 0;

        for (final double[] row : matrix) {
            if (row != null) {
                lengthOfLongestDecimalPart =
                        Math.max(lengthOfLongestDecimalPart,
                                findLengthOfLongestDecimalPart(row));
            }
        }

        return lengthOfLongestDecimalPart;
    }

    /**
     * Finds the length of the longest decimal part present in the {@code row}.
     * If {@code row} is {@code null}, returns 0.
     *
     * @param row the row to search in.
     * @return the length of the longest decimal part.
     */
    private static int findLengthOfLongestDecimalPart(final double[] row) {
        if (row == null) {
            return 0;
        }

        int lengthOfLongestDecimalPart = 0;

        for (final double element : row) {
            final String string = String.valueOf(element);
            final int currentElementDecimalPartLength =
                    string.length() - 1 - string.indexOf('.');

            lengthOfLongestDecimalPart =
                    Math.max(lengthOfLongestDecimalPart,
                            currentElementDecimalPartLength);
        }

        return lengthOfLongestDecimalPart;
    }

    /**
     * Returns an array containing two string arrays. The first array specifies
     * the format strings for the double elements, and the second array
     * specifies the format strings for the string representing missing tokens.
     * Both arrays have the same length, equal to the length of the longest row
     * in the input matrix.
     *
     * In each array, the first string specifies the format for the first matrix
     * column, second string specifies the format for the second matrix column,
     * and so on.
     *
     * @param matrix            the matrix to process.
     * @param maximumRowLength  the maximum row length in the matrix.
     * @param decimalPartLength the requested decimal part length.
     * @return two string arrays describing the formats for the matrix columns.
     */
    private static String[][]
    createColumnFormatStrings(final double[][] matrix,
                              final int maximumRowLength,
                              final int decimalPartLength) {
        final String[] doubleFormatStrings = new String[maximumRowLength];
        final String[] stringFormatStrings = new String[maximumRowLength];
        final String[][] result = new String[2][];

        result[DOUBLE_FORMAT_STRINGS] = doubleFormatStrings;
        result[STRING_FORMAT_STRINGS] = stringFormatStrings;

        final int[] columnIndexToIntegerPartLengthMap =
                getColumnIntegerPartLengths(matrix, maximumRowLength);

        for (int columnIndex = 0;
             columnIndex < doubleFormatStrings.length;
             columnIndex++) {
            final int integerPartLength =
                    columnIndexToIntegerPartLengthMap[columnIndex];

            final String doubleFormat =
                    "%" + (integerPartLength
                            + (decimalPartLength > 0 ? 1 : 0) + decimalPartLength)
                            + "."
                            + decimalPartLength
                            + "f";

            final String stringFormat =
                    "%" + (integerPartLength
                            + (decimalPartLength > 0 ? 1 : 0)
                            + decimalPartLength)
                            + "s";

            doubleFormatStrings[columnIndex] = doubleFormat;
            stringFormatStrings[columnIndex] = stringFormat;
        }

        return result;
    }

    /**
     * Creates a space token used for separating the matrix columns.
     *
     * @param length the number of spaces to use as a separator.
     * @return the column separator string.
     */
    private static String getColumnSeparatorString(final int length) {
        final StringBuilder sb = new StringBuilder(length);

        for (int i = 0; i < length; ++i) {
            sb.append(' ');
        }

        return sb.toString();
    }

    /**
     * Creates a string representing a missing value in the input matrix.
     *
     * @param decimalPartLength the requested length of the decimal parts.
     * @return the string representing a missing matrix value.
     */
    private static String getMissingElementString(final int decimalPartLength) {
        if (decimalPartLength == 0) {
            return "x";
        }

        final StringBuilder sb = new StringBuilder(2 + decimalPartLength);

        sb.append("x.");

        for (int i = 0; i < decimalPartLength; ++i) {
            sb.append('x');
        }

        return sb.toString();
    }

    /**
     * Converts {@code row} to a string representing it.
     *
     * @param sb                   the {@link java.lang.StringBuilder} for
     *                             holding the text data.
     * @param row                  the row to convert to a string.
     * @param formatStringData     the data structure holding the format strings
     *                             for each matrix column.
     * @param separatorString      the string used for separating matrix
     *                             columns.
     * @param missingElementString the string used for denoting a missing
     *                             value in the matrix.
     */
    private static void rowToString(final StringBuilder sb,
                                    final double[] row,
                                    final String[][] formatStringData,
                                    final String separatorString,
                                    final String missingElementString) {
        if (row == null || row.length == 0) {
            sb.append(String.format(formatStringData[STRING_FORMAT_STRINGS][0],
                    missingElementString));
        } else {
            sb.append(String.format(formatStringData[DOUBLE_FORMAT_STRINGS][0],
                    row[0]));
        }

        int x = 1;

        final int boundX = row == null ? 0 : row.length;

        for (; x < boundX; ++x) {
            sb.append(separatorString)
                    .append(String.format(formatStringData[DOUBLE_FORMAT_STRINGS][x],
                            row[x]));
        }

        final int maximumRowLength = formatStringData[0].length;

        for (; x < maximumRowLength; ++x) {
            sb.append(separatorString)
                    .append(String.format(formatStringData[STRING_FORMAT_STRINGS][x],
                            missingElementString));
        }
    }

    /**
     * Returns an array of integers describing the maximum lengths of the
     * integer parts in each column, i.e., {@code result[index]} gives the
     * length of the longest integer part (in digits) in the column indexed by
     * {@code index}.
     *
     * @param matrix           the matrix to process.
     * @param maximumRowLength the maximum row length.
     * @return the array describing the maximum integer part lengths in each
     *         matrix column.
     */
    private static int[]
    getColumnIntegerPartLengths(final double[][] matrix,
                                final int maximumRowLength) {
        final int[] columnIndexToIntegerPartLengthMap =
                new int[maximumRowLength];

        for (int columnIndex = 0;
             columnIndex < maximumRowLength;
             columnIndex++) {
            int currentMaximumIntegerPartLength = 1;

            for (int y = 0; y < matrix.length; ++y) {
                if (matrix[y] != null && columnIndex < matrix[y].length) {
                    final String currentElementString =
                            String.valueOf(matrix[y][columnIndex]);

                    final int currentIntegerPartLength =
                            currentElementString.indexOf('.');

                    currentMaximumIntegerPartLength =
                            Math.max(currentMaximumIntegerPartLength,
                                    currentIntegerPartLength);
                }
            }

            columnIndexToIntegerPartLengthMap[columnIndex] =
                    currentMaximumIntegerPartLength;
        }

        return columnIndexToIntegerPartLengthMap;
    }
}
