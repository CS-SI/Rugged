package org.orekit.rugged.raster;

import org.hipparchus.random.RandomGenerator;
import org.hipparchus.random.Well19937a;
import org.hipparchus.util.ArithmeticUtils;
import org.hipparchus.util.FastMath;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;

/**
 * To simulate DEM SRTM3 V4.0 tiles (without the real data !)
 * The tiles are seamless = no overlapping 
 */
public class DummySRTMelevationUpdater implements TileUpdater {
    
    /** SRTM tile size (deg) */
    private double tileSizeDeg;

    /** SRTM tile number of rows and column */
    private int n;

    /** SRTM tile step (rad) */
    private double stepRad;

    /** Elevations map */
    private double[][] heightMap;

    /**
     * Constructor with dummy elevations to fill in the tile ...
     * @param tileSizeDeg SRTM tile size (deg)
     * @param rowCol SRTM tile number of rows and column (power of 2 plus 1)
     * @param baseH elevation of the base of DEM; unit = m
     * @param initialScale
     * @param reductionFactor for smoothing for low value (0.1) or not (0.5 for instance) the landscape
     * @param seed
     */
    public DummySRTMelevationUpdater(final double tileSizeDeg, final int rowCol, 
                                     double baseH, double initialScale, double reductionFactor, long seed) {
        this.n = rowCol;
        if (!ArithmeticUtils.isPowerOfTwo(n - 1)) {
            throw new RuggedException(RuggedMessages.INTERNAL_ERROR, " : tile size must be a power of two plus one");
        }

        this.tileSizeDeg = tileSizeDeg;
        this.stepRad = FastMath.toRadians(this.tileSizeDeg / (this.n - 1));

        // as we want to use this for testing and comparison purposes,
        // and as we don't control when tiles are generated, we generate
        // only ONCE a height map with continuity at the edges, and
        // reuse this single height map for ALL generated tiles
        heightMap = new double[n][n];
        RandomGenerator random  = new Well19937a(seed);

        // initialize corners in diamond-square algorithm
        heightMap[0][0]         = baseH;
        heightMap[0][n - 1]     = baseH;
        heightMap[n - 1][0]     = baseH;
        heightMap[n - 1][n - 1] = baseH;

        double scale = initialScale;
        for (int span = n - 1; span > 1; span = span / 2) {
            // perform squares step
            for (int i = span / 2; i < n; i += span) {
                for (int j = span / 2; j < n; j += span) {
                    double middleH = mean(i - span / 2, j - span / 2,
                                          i - span / 2, j + span / 2,
                                          i + span / 2, j - span / 2,
                                          i + span / 2, j + span / 2) +
                                    scale * (random.nextDouble() - 0.5);
                    heightMap[i][j] = middleH;
                }
            }

            // perform diamonds step
            boolean flipFlop = false;
            for (int i = 0; i < n - 1; i += span / 2) {
                for (int j = flipFlop ? 0 : span / 2; j < n - 1; j += span) {
                    double middleH = mean(i - span / 2, j,
                                          i + span / 2, j,
                                          i,            j - span / 2,
                                          i,            j + span / 2) +
                                    scale * (random.nextDouble() - 0.5);
                    heightMap[i][j] = middleH;
                    if (i == 0) {
                        heightMap[n - 1][j] = middleH;
                    }
                    if (j == 0) {
                        heightMap[i][n - 1] = middleH;
                    }
                }
                flipFlop = !flipFlop;
            }
            // reduce scale
            scale *= reductionFactor;
        }
    }

    /**
     * Update the tile
     * @param latitude latitude (rad)
     * @param longitude longitude (rad)
     * @param tile to be updated
     */
    public void updateTile(double latitude, double longitude, UpdatableTile tile) {

        // Get the tile indices that contents the current latitude and longitude
        int[] tileIndex = findIndexInTile(latitude, longitude);
        double latIndex = tileIndex[0];
        double lonIndex = tileIndex[1];

        // Init the tile geometry with the Rugged convention: 
        //   the minimum latitude and longitude correspond to the center of the most South-West cell, 
        //   and the maximum latitude and longitude correspond to the center of the most North-East cell.
        // For SRTM : origin latitude is at North of the tile (and latitude step is < 0)
        double minLatitude = FastMath.toRadians(60. - latIndex*5.) + 0.5*stepRad;
        double minLongitude = FastMath.toRadians(-180. + (lonIndex - 1)*5.) + 0.5*stepRad;

        tile.setGeometry(minLatitude, minLongitude, stepRad, stepRad, n, n);
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                tile.setElevation(i, j, heightMap[i][j]);
            }
        }
    }
    
    /**
     * Find tile indices that contain the given (latitude, longitude)
     * @param latitude latitude (rad)
     * @param longitude longitude (rad)
     * @return array : [0] = latitude tile index and [1] = longitude tile index
     */
    private int[] findIndexInTile(double latitude, double longitude) {
        
      double latDeg = FastMath.toDegrees(latitude);
      double lonDeg = FastMath.toDegrees(longitude);
      
      // Compute longitude tile name with longitude value: tile 1 starts at 180 deg W; tile = 72 starts at 175 deg E
      int tileLongIndex = (int) (1 + (lonDeg + 180.)/5.); 

      // Compute latitude tile name with latitude value: tile = 1 starts at 60 deg N; tile = 23 starts at 50 deg S
      int tileLatIndex = (int) (1 + (60. - latDeg)/5.); 

      return new int[] {tileLatIndex, tileLongIndex};
   }
    
    public double getTileStepDeg() {
        return FastMath.toDegrees(this.stepRad);
    }
    
    private double mean(int i1, int j1, int i2, int j2, int i3, int j3, int i4, int j4) {
        return (heightMap[(i1 + n) % n][(j1 + n) % n] +
                        heightMap[(i2 + n) % n][(j2 + n) % n] +
                        heightMap[(i3 + n) % n][(j3 + n) % n] +
                        heightMap[(i4 + n) % n][(j4 + n) % n]) / 4;
    }
}
