/* Copyright 2013-2015 CS Systèmes d'Information
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
package org.orekit.rugged.intersection.duvenhage;

import org.apache.commons.math3.analysis.BivariateFunction;
import org.apache.commons.math3.analysis.function.Max;
import org.apache.commons.math3.analysis.function.Min;
import org.apache.commons.math3.util.FastMath;
import org.orekit.rugged.errors.DumpManager;
import org.orekit.rugged.raster.SimpleTile;

/** Simple implementation of a {@link org.orekit.rugged.raster.Tile}
 * with a min/max kd tree.
 * <p>
 * A n level min/max kd-tree contains sub-tiles merging individual cells
 * together from coarse-grained (at level 0) to fine-grained (at level n-1).
 * Level n-1, which is the deepest one, is computed from the raw cells by
 * merging adjacent cells pairs columns (i.e. cells at indices (i, 2j)
 * and (i, 2j+1) are merged together by computing and storing the minimum
 * and maximum in a sub-tile. Level n-1 therefore has the same number of rows
 * but half the number of columns of the raw tile, and its sub-tiles are
 * 1 cell high and 2 cells wide. Level n-2 is computed from level n-1 by
 * merging sub-tiles rows. Level n-2 therefore has half the number of rows
 * and half the number of columns of the raw tile, and its sub-tiles are
 * 2 cells high and 2 cells wide. Level n-3 is again computed by merging
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
 * <tr BGCOLOR="#EEEEFF">
 *             <td>Level</td>         <td>Number of sub-tiles</td>    <td>Regular sub-tiles dimension</td></tr>
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

 * @see MinMaxTreeTileFactory
 * @author Luc Maisonobe
 */
public class MinMaxTreeTile extends SimpleTile {

    /** Raw elevations. */
    private double[] raw;

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

        raw = elevations;

        final int nbRows = getLatitudeRows();
        final int nbCols = getLongitudeColumns();

        // set up the levels
        final int size = setLevels(0, nbRows, nbCols);
        minTree = new double[size];
        maxTree = new double[size];

        // compute min/max trees
        if (start.length > 0) {

            final double[] preprocessed = new double[raw.length];

            preprocess(preprocessed, raw, nbRows, nbCols, new Min());
            applyRecursively(minTree, start.length - 1, nbRows, nbCols, new Min(), preprocessed, 0);

            preprocess(preprocessed, raw, nbRows, nbCols, new Max());
            applyRecursively(maxTree, start.length - 1, nbRows, nbCols, new Max(), preprocessed, 0);

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
     * <p>
     * Note that the min elevation is <em>not</em> computed
     * only at cell center, but considering that it is interpolated
     * considering also Eastwards and Northwards neighbors, and extends
     * up to the center of these neighbors. As an example, lets consider
     * four neighboring cells in some Digital Elevation Model:
     * <table border="0" cellpadding="5" bgcolor="#f5f5dc">
     * <tr><th bgcolor="#c9d5c9">j+1</th><td>11</td><td>10</td></tr>
     * <tr><th bgcolor="#c9d5c9">j</th><td>12</td><td>11</td></tr>
     * <tr  bgcolor="#c9d5c9"><th>j/i</th><th>i</th><th>i+1</th></tr>
     * </table>
     * When we interpolate elevation at a point located slightly South-West
     * to the center of the (i+1, j+1) cell, we use all four cells in the
     * interpolation, and we will get a result very close to 10 if we start
     * close to (i+1, j+1) cell center. As the min value for this interpolation
     * is stored at (i, j) indices, this implies that {@code getMinElevation(i,
     * j, l)} must return 10 if l is chosen such that the sub-tile at
     * tree level l includes cell (i,j) but not cell (i+1, j+1). In other words,
     * interpolation implies sub-tile boundaries are overshoot by one column to
     * the East and one row to the North when computing min.
     * </p>
     * @param i row index of the cell
     * @param j column index of the cell
     * @param level tree level
     * @return minimum value that can be reached when interpolating elevation
     * in the sub-tile
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

        if (DumpManager.isActive()) {
            final int[] min = locateMin(i, j, level);
            DumpManager.dumpTileCell(this, min[0], min[1],
                                     raw[min[0] * getLongitudeColumns() + min[1]]);
        }

        return minTree[start[level] + levelI * levelC + levelJ];

    }

    /** Get the maximum elevation at some level tree.
     * <p>
     * Note that the max elevation is <em>not</em> computed
     * only at cell center, but considering that it is interpolated
     * considering also Eastwards and Northwards neighbors, and extends
     * up to the center of these neighbors. As an example, lets consider
     * four neighboring cells in some Digital Elevation Model:
     * <table border="0" cellpadding="5" bgcolor="#f5f5dc">
     * <tr><th bgcolor="#c9d5c9">j+1</th><td>11</td><td>12</td></tr>
     * <tr><th bgcolor="#c9d5c9">j</th><td>10</td><td>11</td></tr>
     * <tr  bgcolor="#c9d5c9"><th>j/i</th><th>i</th><th>i+1</th></tr>
     * </table>
     * When we interpolate elevation at a point located slightly South-West
     * to the center of the (i+1, j+1) cell, we use all four cells in the
     * interpolation, and we will get a result very close to 12 if we start
     * close to (i+1, j+1) cell center. As the max value for this interpolation
     * is stored at (i, j) indices, this implies that {@code getMaxElevation(i,
     * j, l)} must return 12 if l is chosen such that the sub-tile at
     * tree level l includes cell (i,j) but not cell (i+1, j+1). In other words,
     * interpolation implies sub-tile boundaries are overshoot by one column to
     * the East and one row to the North when computing max.
     * </p>
     * @param i row index of the cell
     * @param j column index of the cell
     * @param level tree level
     * @return maximum value that can be reached when interpolating elevation
     * in the sub-tile
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

        if (DumpManager.isActive()) {
            final int[] max = locateMax(i, j, level);
            DumpManager.dumpTileCell(this, max[0], max[1],
                                     raw[max[0] * getLongitudeColumns() + max[1]]);
        }

        return maxTree[start[level] + levelI * levelC + levelJ];

    }

    /** Locate the cell at which min elevation is reached for a specified level.
     * <p>
     * Min is computed with respect to the continuous interpolated elevation, which
     * takes four neighboring cells into account. This implies that the cell at which
     * min value is reached for some level is either within the sub-tile for this level,
     * or in some case it may be one column outside to the East or one row outside to
     * the North. See {@link #getMinElevation()} for a more complete explanation.
     * </p>
     * @param i row index of the cell
     * @param j column index of the cell
     * @param level tree level of the sub-tile considered
     * @return row/column indices of the cell at which min elevation is reached
     */
    public int[] locateMin(final int i, final int j, final int level) {

        final int k  = start.length - level;
        int rowShift = k / 2;
        int colShift = (k + 1) / 2;
        int levelI   = i >> rowShift;
        int levelJ   = j >> colShift;
        int levelR   = 1 + ((getLatitudeRows()     - 1) >> rowShift);
        int levelC   = 1 + ((getLongitudeColumns() - 1) >> colShift);

        // track the cell ancestors from merged tree at specified level up to tree at level 1
        for (int l = level + 1; l < start.length; ++l) {

            if (isColumnMerging(l)) {

                --colShift;
                levelC = 1 + ((getLongitudeColumns() - 1) >> colShift);
                levelJ = levelJ << 1;

                if (levelJ + 1 < levelC) {
                    // the cell results from a regular merging of two columns
                    if (minTree[start[l] + levelI * levelC + levelJ] >
                        minTree[start[l] + levelI * levelC + levelJ + 1]) {
                        levelJ++;
                    }
                }

            } else {

                --rowShift;
                levelR = 1 + ((getLatitudeRows() - 1) >> rowShift);
                levelI = levelI << 1;

                if (levelI + 1 < levelR) {
                    // the cell results from a regular merging of two rows
                    if (minTree[start[l] + levelI       * levelC + levelJ] >
                        minTree[start[l] + (levelI + 1) * levelC + levelJ]) {
                        levelI++;
                    }
                }

            }

        }

        // we are now at first merge level, which always results from a column merge
        // or pre-processed data, which themselves result from merging four cells
        // used in interpolation
        // this imply the ancestor of min/max at (n, m) is one of
        // (2n, m), (2n+1, m), (2n+2, m), (2n, m+1), (2n+1, m+1), (2n+2, m+1)
        int minI = levelI;
        int minJ = 2 * levelJ;
        double minElevation = Double.POSITIVE_INFINITY;
        for (int n = 2 * levelJ; n < 2 * levelJ + 3; ++n) {
            if (n < getLongitudeColumns()) {
                for (int m = levelI; m < levelI + 2; ++m) {
                    if (m < getLatitudeRows()) {
                        final double elevation = raw[m * getLongitudeColumns() + n];
                        if (elevation < minElevation) {
                            minI         = m;
                            minJ         = n;
                            minElevation = elevation;
                        }
                    }
                }
            }
        }

        return new int[] {
            minI, minJ
        };

    }

    /** Locate the cell at which max elevation is reached for a specified level.
     * <p>
     * Max is computed with respect to the continuous interpolated elevation, which
     * takes four neighboring cells into account. This implies that the cell at which
     * max value is reached for some level is either within the sub-tile for this level,
     * or in some case it may be one column outside to the East or one row outside to
     * the North. See {@link #getMaxElevation()} for a more complete explanation.
     * </p>
     * @param i row index of the cell
     * @param j column index of the cell
     * @param level tree level of the sub-tile considered
     * @return row/column indices of the cell at which min elevation is reached
     */
    public int[] locateMax(final int i, final int j, final int level) {

        final int k  = start.length - level;
        int rowShift = k / 2;
        int colShift = (k + 1) / 2;
        int levelI   = i >> rowShift;
        int levelJ   = j >> colShift;
        int levelR   = 1 + ((getLatitudeRows()     - 1) >> rowShift);
        int levelC   = 1 + ((getLongitudeColumns() - 1) >> colShift);

        // track the cell ancestors from merged tree at specified level up to tree at level 1
        for (int l = level + 1; l < start.length; ++l) {

            if (isColumnMerging(l)) {

                --colShift;
                levelC = 1 + ((getLongitudeColumns() - 1) >> colShift);
                levelJ = levelJ << 1;

                if (levelJ + 1 < levelC) {
                    // the cell results from a regular merging of two columns
                    if (maxTree[start[l] + levelI * levelC + levelJ] <
                            maxTree[start[l] + levelI * levelC + levelJ + 1]) {
                        levelJ++;
                    }
                }

            } else {

                --rowShift;
                levelR = 1 + ((getLatitudeRows() - 1) >> rowShift);
                levelI = levelI << 1;

                if (levelI + 1 < levelR) {
                    // the cell results from a regular merging of two rows
                    if (maxTree[start[l] + levelI       * levelC + levelJ] <
                            maxTree[start[l] + (levelI + 1) * levelC + levelJ]) {
                        levelI++;
                    }
                }

            }

        }

        // we are now at first merge level, which always results from a column merge
        // or pre-processed data, which themselves result from merging four cells
        // used in interpolation
        // this imply the ancestor of min/max at (n, m) is one of
        // (2n, m), (2n+1, m), (2n+2, m), (2n, m+1), (2n+1, m+1), (2n+2, m+1)
        int maxI = levelI;
        int maxJ = 2 * levelJ;
        double maxElevation = Double.NEGATIVE_INFINITY;
        for (int n = 2 * levelJ; n < 2 * levelJ + 3; ++n) {
            if (n < getLongitudeColumns()) {
                for (int m = levelI; m < levelI + 2; ++m) {
                    if (m < getLatitudeRows()) {
                        final double elevation = raw[m * getLongitudeColumns() + n];
                        if (elevation > maxElevation) {
                            maxI         = m;
                            maxJ         = n;
                            maxElevation = elevation;
                        }
                    }
                }
            }
        }

        return new int[] {
            maxI, maxJ
        };

    }

    /** Get the deepest level at which two cells are merged in the same min/max sub-tile.
     * @param i1 row index of first cell
     * @param j1 column index of first cell
     * @param i2 row index of second cell
     * @param j2 column index of second cell
     * @return deepest level at which two cells are merged in the same min/max sub-tile,
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
     * the endpoints <em>are</em> included (i.e. if {@code row1} or {@code row2} are
     * boundary rows, they will be in returned array)
     */
    public int[] getCrossedBoundaryRows(final int row1, final int row2, final int level) {

        // number of rows in each sub-tile
        final int rows = 1 << ((start.length - level) / 2);

        // build the crossings in ascending order
        final int min = FastMath.min(row1, row2);
        final int max = FastMath.max(row1, row2) + 1;
        return buildCrossings((min + rows - 1) - ((min + rows - 1) % rows), max, rows,
                              row1 <= row2);

    }

    /** Get the index of sub-tiles start columns crossed.
     * <p>
     * When going from one column to another column at some tree level,
     * we cross sub-tiles boundaries. This method returns the index
     * of these boundaries.
     * </p>
     * @param column1 starting column
     * @param column2 ending column (excluded)
     * @param level tree level
     * @return indices of columns crossed at sub-tiles boundaries, in crossing order,
     * the endpoints <em>are</em> included (i.e. if {@code column1} or {@code column2} are
     * boundary columns, they will be in returned array)
     */
    public int[] getCrossedBoundaryColumns(final int column1, final int column2, final int level) {

        // number of columns in each sub-tile
        final int columns  = 1 << ((start.length + 1 - level) / 2);

        // build the crossings in ascending order
        final int min = FastMath.min(column1, column2);
        final int max = FastMath.max(column1, column2) + 1;
        return buildCrossings((min + columns - 1) - ((min + columns - 1) % columns), max, columns,
                              column1 <= column2);

    }

    /** Build crossings arrays.
     * @param begin begin crossing index
     * @param end end crossing index (excluded, if equal to begin, the array is empty)
     * @param step crossing step
     * @param ascending if true, the crossings must be in ascending order
     * @return indices of rows or columns crossed at sub-tiles boundaries, in crossing order
     */
    private int[] buildCrossings(final int begin, final int end, final int step, final boolean ascending) {

        // allocate array
        final int n = FastMath.max(0, (end - begin + step + ((step > 0) ? -1 : +1)) / step);
        final int[] crossings = new int[n];

        // fill it up
        int crossing = begin;
        if (ascending) {
            for (int i = 0; i < crossings.length; ++i) {
                crossings[i] = crossing;
                crossing += step;
            }
        } else {
            for (int i = 0; i < crossings.length; ++i) {
                crossings[crossings.length - 1 - i] = crossing;
                crossing += step;
            }
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
     * <tr BGCOLOR="#EEEEFF">
     *     <td>Level</td>   <td>Dimension</td>  <td>Start index</td>  <td>End index (inclusive)</td></tr>
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

    /** Preprocess recursive application of a function.
     * <p>
     * At start, the min/max should be computed for each cell using the four corners values.
     * </p>
     * @param preprocessed preprocessed array to fill up
     * @param elevations raw elevations te preprocess
     * @param nbRows number of rows
     * @param nbCols number of columns
     * @param f function to apply
     */
    private void preprocess(final double[] preprocessed, final double[] elevations,
                            final int nbRows, final int nbCols,
                            final BivariateFunction f) {

        int k = 0;

        for (int i = 0; i < nbRows - 1; ++i) {

            // regular elements with both a column at right and a row below
            for (int j = 0; j < nbCols - 1; ++j) {
                preprocessed[k] = f.value(f.value(elevations[k],          elevations[k + 1]),
                                          f.value(elevations[k + nbCols], elevations[k + nbCols + 1]));
                k++;
            }

            // last column elements, lacking a right column
            preprocessed[k] = f.value(elevations[k], elevations[k + nbCols]);
            k++;

        }

        // last row elements, lacking a below row
        for (int j = 0; j < nbCols - 1; ++j) {
            preprocessed[k] = f.value(elevations[k], elevations[k + 1]);
            k++;
        }

        // last element
        preprocessed[k] = elevations[k];

    }

    /** Recursive application of a function.
     * @param tree tree to fill-up with the recursive applications
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
            final int     jEnd        = odd ? nextColumns - 1 : nextColumns;
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
            final int     iEnd     = odd ? nextRows - 1 : nextRows;

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
