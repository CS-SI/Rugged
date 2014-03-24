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
package org.orekit.rugged.core.raster;

import org.apache.commons.math3.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.api.TileUpdater;
import org.orekit.rugged.api.UpdatableTile;
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

    public void updateTile(double latitude, double longitude, UpdatableTile tile)
        throws RuggedException {
        double step         = size / (n - 1);
        double minLatitude  = size * FastMath.floor(latitude  / size);
        double minLongitude = size * FastMath.floor(longitude / size);
        double sinSlope     = FastMath.sin(slope);
        tile.setGeometry(minLatitude, minLongitude, step, step, n, n);
        for (int i = 0; i < n; ++i) {
            double pixelLatitude = minLatitude + i * step;
            for (int j = 0; j < n; ++j) {
                double pixelLongitude = minLongitude + j * step;
                double distance       = Constants.WGS84_EARTH_EQUATORIAL_RADIUS *
                                        FastMath.hypot(pixelLatitude  - summit.getLatitude(),
                                                       pixelLongitude - summit.getLongitude());
                double altitude = FastMath.max(summit.getAltitude() - distance * sinSlope,
                                               base);
                tile.setElevation(i, j, altitude);
            }
        }
    }

}
