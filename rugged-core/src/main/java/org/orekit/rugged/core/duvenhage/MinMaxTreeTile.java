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
import org.apache.commons.math3.util.FastMath;
import org.orekit.rugged.core.raster.SimpleTile;
import org.orekit.rugged.core.raster.Tile;

/** Simple implementation of a {@link Tile} with a min/max kd tree.
 * <p>
 * A n level min/max kd-tree contains sub-tiles merging individual pixels
 * together from coarse-grained (at level 0) to fine-grained (at level n-1).
 * Level n-1, which is the deepest one, is computed from the raw pixels by
 * merging adjacent pixels pairs columns (i.e. pixels at indices (i, 2j)
 * and (i, 2j+1) are merged together by computing and storing the minimum
 * and maxium in a sub-tile. Level n-1 therefore has the same number of rows
 * but half the number of columns of the raw tile, and its sub-tiles are
 * 1 pixel high and 2 pixels wide. Level n-2 is computed from level n-1 by
 * merging sub-tiles rows. Level n-2 therefore has half the number of rows
 * and half the number of columns of the raw tile, and its sub-tiles are
 * 2 pixels high and 2 pixels wide. Level n-3 is again computed by merging
 * columns, level n-4 merging rows and so on. As depth decreases, the number
 * of sub-tiles decreases and their size increase. Level 0 is reached when
 * there is only either one row or one column of large sub-tiles.
 * </p>
 * <p>
 * During the merging process, if at some stage there is an odd number of
 * rows or columns, then the last sub-tile at next level will not be computed
 * by merging two rows/columns from the current level, but instead computed by
 * simply copying the last single row/column. The process is therefore well
 * defined for any raw tile initial dimensions. A direct consequence is that
 * the dimension of the sub-tiles in the last row or column may be smaller than
 * the dimension of regular sub-tiles.
 * </p>
 * <p>
 * If we consider for example a tall 107 ⨉ 19 raw tile, the min/max kd-tree will
 * have 9 levels:
 * </p>
 * <p>
 * <table border="0">
 * <tr BGCOLOR="#EEEEFF"><font size="+1">
 *             <td>Level</td>         <td>Number of sub-tiles</td>    <td>Regular sub-tiles dimension</td></font></tr>
 * <tr>  <td align="center">8</td>  <td align="center">107 ⨉ 10</td>       <td align="center"> 1 ⨉   2</td> 
 * <tr>  <td align="center">7</td>  <td align="center"> 54 ⨉ 10</td>       <td align="center"> 2 ⨉   2</td> 
 * <tr>  <td align="center">6</td>  <td align="center"> 54 ⨉  5</td>        <td align="center"> 2 ⨉  4</td>
 * <tr>  <td align="center">5</td>  <td align="center"> 27 ⨉  5</td>        <td align="center"> 4 ⨉  4</td>
 * <tr>  <td align="center">4</td>  <td align="center"> 27 ⨉  3</td>        <td align="center"> 4 ⨉  8</td>
 * <tr>  <td align="center">3</td>  <td align="center"> 14 ⨉  3</td>        <td align="center"> 8 ⨉  8</td>
 * <tr>  <td align="center">2</td>  <td align="center"> 14 ⨉  2</td>        <td align="center"> 8 ⨉ 16</td>
 * <tr>  <td align="center">1</td>  <td align="center">  7 ⨉  2</td>        <td align="center">16 ⨉ 16</td>
 * <tr>  <td align="center">0</td>  <td align="center">  7 ⨉  1</td>        <td align="center">16 ⨉ 32</td>
 * </table>
 * </p>

 * </p>
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

    /** Get the deepest level at which two pixels are merged in the same min/max sub-tile.
     * @param i1 row index of first pixel
     * @param j1 column index of first pixel
     * @param i2 row index of second pixel
     * @param j2 column index of second pixel
     * @return deepest level at which two pixels are merged in the same min/max sub-tile,
     * or -1 if they are never merged in the same sub-tile
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

    /** Get the index of sub-tiles start rows crossed.
     * <p>
     * When going from one row to another row at some tree level,
     * we cross sub-tiles boundaries. This method returns the index
     * of these boundaries.
     * </p>
     * @param row1 starting row
     * @param row2 ending row
     * @param level tree level
     * @return indices of rows crossed at sub-tiles boundaries, in crossing order,
     * excluding the start and end rows themselves even if they are at a sub-tile boundary
     */
    public int[] getCrossedBoundaryRows(final int row1, final int row2, final int level) {

        // number of rows in each sub-tile
        final int rows  = 1 << ((start.length - level) / 2);

        if (row1 <= row2) {
            final int nextMultiple = row1 + rows - (row1 % rows);
            return buildCrossings(nextMultiple, row2, rows);
        } else {
            final int previousMultiple = row1 - 1 - ((row1 - 1) % rows);
            return buildCrossings(previousMultiple, row2, -rows);
        }

    }

    /** Get the index of sub-tiles start columns crossed.
     * <p>
     * When going from one column to another column at some tree level,
     * we cross sub-tiles boundaries. This method returns the index
     * of these boundaries.
     * </p>
     * @param column1 starting column
     * @param column2 ending column
     * @param level tree level
     * @return indices of columns crossed at sub-tiles boundaries, in crossing order
     * excluding the start and end columns themselves even if they are at a sub-tile boundary
     */
    public int[] getCrossedBoundaryColumns(final int column1, final int column2, final int level) {

        // number of columns in each sub-tile
        final int columns  = 1 << ((start.length + 1 - level) / 2);;

        if (column1 <= column2) {
            final int nextMultiple = column1 + columns - (column1 % columns);
            return buildCrossings(nextMultiple, column2,  columns);
        } else {
            final int previousMultiple = column1 - 1 - ((column1 - 1) % columns);
            return buildCrossings(previousMultiple, column2, -columns);
        }

    }

    /** Build crossings arrays.
     * @param begin begin crossing index
     * @param end end crossing index (excluded, if equal to begin, the array is empty)
     * @param step crossing step (may be negative)
     * @return indices of rows or columns crossed at sub-tiles boundaries, in crossing order
     */
    private int[] buildCrossings(final int begin, final int end, final int step) {

        // allocate array
        final int n = FastMath.max(0, (end - begin + step + ((step > 0) ? -1 : +1)) / step);
        final int[] crossings = new int[n];

        // fill it up
        int crossing = begin;
        for (int i = 0; i < crossings.length; ++i) {
            crossings[i] = crossing;
            crossing    += step;
        }

        return crossings;

    }

    /** Check if the merging operation between level and level-1 is a column merging.
     * @param level level to check
     * @return true if the merging operation between level and level-1 is a column
     * merging, false if is a row merging
     */
    public boolean isColumnMerging(final int level) {
        return (level & 0x1) == (start.length & 0x1);
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

        if (isColumnMerging(level + 1)) {

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
