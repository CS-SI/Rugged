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
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.api.TileUpdater;
import org.orekit.rugged.api.UpdatableTile;

public class CheckedPatternElevationUpdater implements TileUpdater {

    private double size;
    private int    n;
    private double elevation1;
    private double elevation2;

    public CheckedPatternElevationUpdater(double size, int n, double elevation1, double elevation2) {
        this.size       = size;
        this.n          = n;
        this.elevation1 = elevation1;
        this.elevation2 = elevation2;
    }

    public void updateTile(double latitude, double longitude, UpdatableTile tile)
        throws RuggedException {
        double step         = size / (n - 1);
        double minLatitude  = size * FastMath.floor(latitude  / size);
        double minLongitude = size * FastMath.floor(longitude / size);
        tile.setGeometry(minLatitude, minLongitude, step, step, n, n);
        for (int i = 0; i < n; ++i) {
            int p = (int) FastMath.floor((minLatitude + i * step) / step);
            for (int j = 0; j < n; ++j) {
                int q = (int) FastMath.floor((minLongitude + j * step) / step);
                tile.setElevation(i, j, (((p ^ q) & 0x1) == 0) ? elevation1 : elevation2);
            }
        }
    }

}
