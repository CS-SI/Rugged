package org.orekit.rugged.raster;

import org.hipparchus.util.FastMath;

/**
 * To simulate DEM SRTM3 V4.0 tiles (without the real data !)
 * The tiles are seamless = no overlapping 
 */
public class DummySRTMsimpleElevationUpdater implements TileUpdater {
    
    /** SRTM tile size (deg) */
    private double tileSizeDeg;

    /** SRTM tile number of rows and column */
    private int n;

    /** SRTM tile step (rad) */
    private double stepRad;

    private double elevation1;

    private double elevation2;
    
    /**
     * Constructor with dummy elevations to fill in the tile ...
     * @param tileSizeDeg SRTM tile size (deg)
     * @param rowCol SRTM tile number of rows and column
     * @param elevation1
     * @param elevation2
     */
    public DummySRTMsimpleElevationUpdater(final double tileSizeDeg, final int rowCol, final double elevation1, final double elevation2) {
        
        this.tileSizeDeg = tileSizeDeg;
        this.n = rowCol;
        this.stepRad = FastMath.toRadians(this.tileSizeDeg / this.n);
        
        this.elevation1 = elevation1;
        this.elevation2 = elevation2;
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
            int p = (int) FastMath.floor((FastMath.abs(minLatitude) + i * stepRad) / stepRad);
            for (int j = 0; j < n; ++j) {
                int q = (int) FastMath.floor((FastMath.abs(minLongitude) + j * stepRad) / stepRad);
                double factor = FastMath.sin(minLatitude + i*stepRad - (minLongitude + j*stepRad))*
                                FastMath.cos(minLatitude + i*stepRad + (minLongitude + j*stepRad));
                tile.setElevation(i, j, (((p ^ q) & 0x1) == 0) ? factor*lonIndex*elevation1 : factor*latIndex*elevation2);
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
}