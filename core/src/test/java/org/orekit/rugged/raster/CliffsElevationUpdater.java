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
package org.orekit.rugged.raster;

import org.apache.commons.math3.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.api.RuggedException;

public class CliffsElevationUpdater implements TileUpdater {

    private GeodeticPoint point1;
    private GeodeticPoint point2;
    private double        top;
    private double        bottom;
    private double        size;
    private int           n;

    public CliffsElevationUpdater(GeodeticPoint point1, GeodeticPoint point2,
                                  double top, double bottom,
                                  double size, int n) {
        this.point1 = point1;
        this.point2 = point2;
        this.top    = top;
        this.bottom = bottom;
        this.size   = size;
        this.n      = n;
    }

    public void updateTile(double latitude, double longitude, UpdatableTile tile)
        throws RuggedException {
        double step         = size / (n - 1);
        double minLatitude  = size * FastMath.floor(latitude  / size);
        double minLongitude = size * FastMath.floor(longitude / size);
        double x2Mx1        = point2.getLongitude() - point1.getLongitude();
        double y2My1        = point2.getLatitude()  - point1.getLatitude();
        tile.setGeometry(minLatitude, minLongitude, step, step, n, n);
        for (int i = 0; i < n; ++i) {
            double cellLatitude = minLatitude + i * step;
            for (int j = 0; j < n; ++j) {
                double cellLongitude = minLongitude + j * step;
                double xMx1  = cellLongitude - point1.getLongitude();
                double yMy1  = cellLatitude  - point1.getLatitude();
                if (yMy1 * x2Mx1 > xMx1 * y2My1) {
                    // left side of the point1 to point2 track
                    tile.setElevation(i, j, top);
                } else {
                    // right side of the point1 to point2 track
                    tile.setElevation(i, j, bottom);
                }
            }
        }
    }

}
