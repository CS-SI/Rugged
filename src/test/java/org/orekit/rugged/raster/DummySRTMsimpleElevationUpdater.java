/* Copyright 2013-2025 CS GROUP
 * Licensed to CS GROUP (CS) under one or more
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
package org.orekit.rugged.raster;

import org.hipparchus.util.FastMath;
import org.hipparchus.util.MathUtils;

/**
 * To simulate DEM SRTM3 V4.0 tiles (without the real data !)
 * The tiles are seamless = no overlapping 
 * 
 * @author Guylaine Prat
 */
public class DummySRTMsimpleElevationUpdater implements TileUpdater {
    
    /** SRTM tile size (deg) */
    private double tileSizeDeg = 5.;

    /** SRTM tile step (rad) */
    private double stepRad;

    /** SRTM tile number of rows and columns */
    private int n;
    
    /** Factor to change resolution above 60 degrees and below 60 degrees */
    private int resolutionChange;

    private double elevation1;
    private double elevation2;
    
    /**
     * Constructor with dummy elevations to fill in the tile ...
     * @param rowCol SRTM tile number of rows and column
     * @param elevation1 chosen elevation1 (m)
     * @param elevation2 chosen elevation2 (m)
     * @param resolutionChange factor to change resolution at +/- 60 degrees
     */
    public DummySRTMsimpleElevationUpdater(final int rowCol, final double elevation1, final double elevation2, final int resolutionChange) {
        
        this.n = rowCol;
        this.stepRad = FastMath.toRadians(this.tileSizeDeg / this.n);
        this.resolutionChange = resolutionChange;
        
        this.elevation1 = elevation1;
        this.elevation2 = elevation2;
    }

    /**
     * Update the tile
     * @param latitude latitude (rad)
     * @param longitude longitude (rad)
     * @param tile to be updated
     */
    public void updateTile(final double latitude, final double longitude, UpdatableTile tile) {

        int numberOfStep;
        
        // Change the tile step for latitude above 60 degrees and below 60 degrees
        if (FastMath.toDegrees(latitude) > 60. || FastMath.toDegrees(latitude) < -60.) {
            // step * resolutionChange for latitude > 60 or < -60
            this.stepRad = FastMath.toRadians(this.tileSizeDeg*this.resolutionChange / this.n);
            numberOfStep = this.n/this.resolutionChange;

        } else { // step = tile size / n
            this.stepRad = FastMath.toRadians(this.tileSizeDeg / this.n);
            numberOfStep = this.n;
        }
        
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

        tile.setGeometry(minLatitude, minLongitude, stepRad, stepRad, numberOfStep, numberOfStep);

        for (int i = 0; i < numberOfStep; ++i) {
            int p = (int) FastMath.floor((FastMath.abs(minLatitude) + i * stepRad) / stepRad);
            for (int j = 0; j < numberOfStep; ++j) {
                int q = (int) FastMath.floor((FastMath.abs(minLongitude) + j * stepRad) / stepRad);
                double factor = FastMath.sin(minLatitude + i*stepRad - (minLongitude + j*stepRad))*
                                FastMath.cos(minLatitude + i*stepRad + (minLongitude + j*stepRad));
                tile.setElevation(i, j, (((p ^ q) & 0x1) == 0) ? factor*lonIndex*elevation1 + q  : factor*latIndex*elevation2 + q*p);
            }
        }
    }
    
    /**
     * Find tile indices that contain the given (latitude, longitude)
     * @param latitude latitude (rad)
     * @param longitude longitude (rad)
     * @return array : [0] = latitude tile index and [1] = longitude tile index
     */
    private int[] findIndexInTile(final double latitude, final double longitude) {
        
      double latDeg = FastMath.toDegrees(latitude);
      
      // Assure that the longitude belongs to [-180, + 180]
      double lonDeg = FastMath.toDegrees(MathUtils.normalizeAngle(longitude, 0.0));

      // Compute longitude tile name with longitude value: 
      // For SRTM with 5 degrees tiles: tile longitude index 1 starts at 180 deg W; tile longitude index 72 starts at 175 deg E
      int tileLongIndex = (int) (1 + (lonDeg + 180.)/5.); 

      // Compute latitude tile name with latitude value: 
      // For SRTM with 5 degrees tiles: tile latitude index 1 starts at 60 deg N; tile latitude index 23 starts at 50 deg S
      int tileLatIndex = (int) (1 + (60. - latDeg)/5.); 

      return new int[] {tileLatIndex, tileLongIndex};
    }
    
    public double getTileStepDeg() {
        return FastMath.toDegrees(this.stepRad);
    }
    
    public double getTileSizeDeg() {
        return this.tileSizeDeg;
    }
}
