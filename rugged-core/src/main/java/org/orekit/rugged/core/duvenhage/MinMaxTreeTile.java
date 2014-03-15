/* Copyright 2013-2014 CS Systèmes d'Information
 * Licensed to CS Systèmes d'Information (CS) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * CS licenses this file to You under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package org.orekit.rugged.core.duvenhage;

import org.apache.commons.math3.analysis.BivariateFunction;
import org.apache.commons.math3.analysis.function.Max;
import org.apache.commons.math3.analysis.function.Min;
import org.orekit.rugged.core.dem.SimpleTile;
import org.orekit.rugged.core.dem.Tile;

/** Simple implementation of a {@link Tile} with a min/max kd tree.
 * @see MinMaxTreeTileFactory
 * @author Luc Maisonobe
 */
public class MinMaxTreeTile extends SimpleTile {

    /** Min kd-tree. */
    private double[] minTree;

    /** Max kd-tree. */
    private double[] maxTree;

    /** Start indices of tree levels. */
    private int[] start;

    /** Simple constructor.
     * <p>
     * Creates an empty tile.
     * </p>
     */
    MinMaxTreeTile() {
    }

    /** {@inheritDoc} */
    @Override
    protected void processUpdatedElevation(final double[] elevations) {

        final int nbRows = getLatitudeRows();
        final int nbCols = getLongitudeColumns();

        // set up the levels
        final int size = setLevels(0, nbRows, nbCols);
        minTree = new double[size];
        maxTree = new double[size];

        // compute min/max trees
        if (start.length > 0) {
            applyRecursively(minTree, start.length - 1, nbRows, nbCols, new Min(), elevations, 0);
            applyRecursively(maxTree, start.length - 1, nbRows, nbCols, new Max(), elevations, 0);
        }

    }

    /** Get the number of kd-tree levels (not counting raw elevations).
     * @return number of kd-tree levels
     * @see #getMinElevation(int, int, int)
     * @see #getMaxElevation(int, int, int)
     * @see #getMergeLevel(int, int, int, int)
     */
    public int getLevels() {
        return start.length;
    }

    /** Get the minimum elevation at some level tree.
     * @param i row index of the pixel
     * @param j column index of the pixel
     * @param level tree level
     * @return minimum elevation
     * @see #getLevels()
     * @see #getMaxElevation(int, int, int)
     * @see #getMergeLevel(int, int, int, int)
     */
    public double getMinElevation(final int i, final int j, final int level) {

        // compute indices in level merged array
        final int k        = start.length - level;
        final int rowShift = k / 2;
        final int colShift = (k + 1) / 2;
        final int levelI   = i >> rowShift;
        final int levelJ   = j >> colShift;
        final int levelC   = 1 + ((getLongitudeColumns() - 1) >> colShift);

        return minTree[start[level] + levelI * levelC + levelJ];

    }

    /** Get the maximum elevation at some level tree.
     * @param i row index of the pixel
     * @param j column index of the pixel
     * @param level tree level
     * @return maximum elevation
     * @see #getLevels()
     * @see #getMinElevation(int, int, int)
     * @see #getMergeLevel(int, int, int, int)
     */
    public double getMaxElevation(final int i, final int j, final int level) {

        // compute indices in level merged array
        final int k        = start.length - level;
        final int rowShift = k / 2;
        final int colShift = (k + 1) / 2;
        final int levelI   = i >> rowShift;
        final int levelJ   = j >> colShift;
        final int levelC   = 1 + ((getLongitudeColumns() - 1) >> colShift);

        return maxTree[start[level] + levelI * levelC + levelJ];

    }

    /** Get the largest level at which two pixels are merged in the same min/max sub-tile.
     * @param i1 row index of first pixel
     * @param j1 column index of first pixel
     * @param i2 row index of second pixel
     * @param j2 column index of second pixel
     * @return largest level at which two pixels are merged in the same min/max sub-tile,
     * or negative if they are never merged in the same sub-tile
     * @see #getLevels()
     * @see #getMinElevation(int, int, int)
     * @see #getMaxElevation(int, int, int)
     */
    public int getMergeLevel(final int i1, final int j1, final int i2, final int j2) {

        int largest = -1;

        for (int level = 0; level < start.length; ++level) {
            // compute indices in level merged array
            final int k        = start.length - level;
            final int rowShift = k / 2;
            final int colShift = (k + 1) / 2;
            final int levelI1  = i1 >> rowShift;
            final int levelJ1  = j1 >> colShift;
            final int levelI2  = i2 >> rowShift;
            final int levelJ2  = j2 >> colShift;
            if (levelI1 != levelI2 || levelJ1 != levelJ2) {
                return largest;
            }
            largest = level;
        }

        return largest;

    }

    /** Check if the merging operation between level and level+1 is a column merging.
     * @param level level to check
     * @return true if the merging operation between level and level+1 is a column
     * merging, false if is a row merging
     */
    public boolean isColumnMerging(final int level) {
        return (level & 0x1) != (start.length & 0x1);
    }

    /** Recursive setting of tree levels.
     * <p>
     * The following algorithms works for any array shape, even with
     * rows or columns which are not powers of 2 or with one
     * dimension much larger than the other. As an example, starting
     * from a 107 ⨉ 19 array, we get the following 9 levels, for a
     * total of 2187 elements in each tree:
     * </p>
     * <p>
     * <table border="0">
     * <tr BGCOLOR="#EEEEFF"><font size="+1">
     *     <td>Level</td>   <td>Dimension</td>  <td>Start index</td>  <td>End index (inclusive)</td></font></tr>
     * <tr>   <td>0</td>     <td>  7 ⨉  1</td>       <td>   0</td>        <td>  6</td> </tr>
     * <tr>   <td>1</td>     <td>  7 ⨉  2</td>       <td>   7</td>        <td> 20</td> </tr>
     * <tr>   <td>2</td>     <td> 14 ⨉  2</td>       <td>  21</td>        <td> 48</td> </tr>
     * <tr>   <td>3</td>     <td> 14 ⨉  3</td>       <td>  49</td>        <td> 90</td> </tr>
     * <tr>   <td>4</td>     <td> 27 ⨉  3</td>       <td>  91</td>        <td>171</td> </tr>
     * <tr>   <td>5</td>     <td> 27 ⨉  5</td>       <td> 172</td>        <td>306</td> </tr>
     * <tr>   <td>6</td>     <td> 54 ⨉  5</td>       <td> 307</td>        <td>576</td> </tr>
     * <tr>   <td>7</td>     <td> 54 ⨉ 10</td>      <td> 577</td>        <td>1116</td> </tr>
     * <tr>   <td>8</td>     <td>107 ⨉ 10</td>      <td>1117</td>        <td>2186</td> </tr>
     * </table>
     * </p>
     * @param stage number of merging stages
     * @param stageRows number of rows at current stage
     * @param stageColumns number of columns at current stage
     * @return size cumulative size from root to current level
     */
    private int setLevels(final int stage, final int stageRows, final int stageColumns) {

        if (stageRows == 1 || stageColumns == 1) {
            // we have found root, stop recursion
            start   = new int[stage];
            if (stage > 0) {
                start[0]   = 0;
            }
            return stageRows * stageColumns;
        }

        final int size;
        if ((stage & 0x1) == 0) {
            // columns merging
            size = setLevels(stage + 1, stageRows, (stageColumns + 1) / 2);
        } else {
            // rows merging
            size = setLevels(stage + 1, (stageRows + 1) / 2, stageColumns);
        }

        if (stage > 0) {
            // store current level characteristics
            start[start.length     - stage] = size;
            return size + stageRows * stageColumns;
        } else {
            // we don't count the elements at stage 0 as they are not stored in the
            // min/max trees (they correspond to the raw elevation, without merging)
            return size;
        }

    }

    /** Recursive application of a function.
     * @param tree to fill-up with the recursive applications
     * @param level current level
     * @param levelRows number of rows at current level
     * @param levelColumns number of columns at current level
     * @param f function to apply
     * @param base base array from which function arguments are drawn
     * @param first index of the first element to consider in base array
     */
    private void applyRecursively(final double[] tree,
                                  final int level, final int levelRows, final int levelColumns,
                                  final BivariateFunction f,
                                  final double[] base, final int first) {

        if (isColumnMerging(level)) {

            // merge columns pairs
            int           iTree       = start[level];
            int           iBase       = first;
            final int     nextColumns = (levelColumns + 1) / 2;
            final boolean odd         = (levelColumns & 0x1) != 0;
            int           jEnd        = odd ? nextColumns - 1 : nextColumns;
            for (int i = 0; i < levelRows; ++i) {

                // regular pairs
                for (int j = 0; j < jEnd; ++j) {
                    tree[iTree++] = f.value(base[iBase], base[iBase + 1]);
                    iBase += 2;
                }

                if (odd) {
                    // last column
                    tree[iTree++] = base[iBase++];
                }


            }

            if (level > 0) {
                applyRecursively(tree, level - 1, levelRows, nextColumns, f, tree, start[level]);
            }

        } else {

            // merge rows pairs
            int           iTree    = start[level];
            int           iBase    = first;
            final int     nextRows = (levelRows + 1) / 2;
            final boolean odd      = (levelRows & 0x1) != 0;
            int           iEnd     = odd ? nextRows - 1 : nextRows;

            // regular pairs
            for (int i = 0; i < iEnd; ++i) {

                for (int j = 0; j < levelColumns; ++j) {
                    tree[iTree++] = f.value(base[iBase], base[iBase + levelColumns]);
                    iBase++;
                }
                iBase += levelColumns;

            }

            if (odd) {
                // last row
                System.arraycopy(base, iBase, tree, iTree, levelColumns);
            }

            if (level > 0) {
                applyRecursively(tree, level - 1, nextRows, levelColumns, f, tree, start[level]);
            }

        }
    }

}
