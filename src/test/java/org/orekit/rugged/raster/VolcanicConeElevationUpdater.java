/* Copyright 2013-2022 CS GROUP
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
import org.orekit.bodies.GeodeticPoint;
import org.orekit.utils.Constants;

public class VolcanicConeElevationUpdater implements TileUpdater {

    private GeodeticPoint summit;
    private double        slope;
    private double        base;
    private double        size;
    private int           n;

    public VolcanicConeElevationUpdater(GeodeticPoint summit, double slope, double base,
                                        double size, int n) {
        this.summit = summit;
        this.slope  = slope;
        this.base   = base;
        this.size   = size;
        this.n      = n;
    }

    public void updateTile(double latitude, double longitude, UpdatableTile tile) {
        
        double step         = size / (n - 1);
        double minLatitude  = size * FastMath.floor(latitude  / size);
        double minLongitude = size * FastMath.floor(longitude / size);
        double sinSlope     = FastMath.sin(slope);
        tile.setGeometry(minLatitude, minLongitude, step, step, n, n);
        for (int i = 0; i < n; ++i) {
            double cellLatitude = minLatitude + i * step;
            for (int j = 0; j < n; ++j) {
                double cellLongitude = minLongitude + j * step;
                double distance       = Constants.WGS84_EARTH_EQUATORIAL_RADIUS *
                                        FastMath.hypot(cellLatitude  - summit.getLatitude(),
                                                       cellLongitude - summit.getLongitude());
                double altitude = FastMath.max(summit.getAltitude() - distance * sinSlope,
                                               base);
                tile.setElevation(i, j, altitude);
            }
        }
    }
}
