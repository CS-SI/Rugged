/* Copyright 2013-2016 CS Systèmes d'Information
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
package org.orekit.rugged.atmosphericrefraction;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.orekit.rugged.raster.Tile;
import org.orekit.rugged.utils.ExtendedEllipsoid;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;

import java.util.Collections;
import java.util.Map;
import java.util.TreeMap;

/**
 * Multi layer model for atmospheric refraction.
 * @author Sergio Esteves
 */
public class MultiLayerModel implements AtmosphericRefraction {

    // maps altitude (lower bound) to refraction index
    private static final Map<Double, Double> meanAtmoshpericRefractions;

    static {
        Map<Double, Double> refractions = new TreeMap(Collections.reverseOrder());
        refractions.put(-1000.00000000000, 1.00030600000);
        refractions.put(0.00000000000, 1.00027800000);
        refractions.put(1000.00000000000, 1.00025200000);
        refractions.put(3000.00000000000, 1.00020600000);
        refractions.put(5000.00000000000, 1.00016700000);
        refractions.put(7000.00000000000, 1.00013400000);
        refractions.put(9000.00000000000, 1.00010600000);
        refractions.put(11000.00000000000, 1.00008300000);
        refractions.put(14000.00000000000, 1.00005200000);
        refractions.put(18000.00000000000, 1.00002800000);
        refractions.put(23000.00000000000, 1.00001200000);
        refractions.put(30000.00000000000, 1.00000400000);
        refractions.put(40000.00000000000, 1.00000100000);
        refractions.put(50000.00000000000, 1.00000000000);
        refractions.put(100000.00000000000, 1.00000000000);
        meanAtmoshpericRefractions = Collections.unmodifiableMap(refractions);
    }

    @Override
    public double getDeviation(Vector3D pos, Vector3D los, Vector3D zenith, double altitude, Tile tile) {

        new ExtendedEllipsoid(ellipsoid.getEquatorialRadius(), ellipsoid.getFlattening(),
                ellipsoid.getBodyFrame());

        double incidenceAngleSin = FastMath.sin(Vector3D.angle(los, zenith));
        double previousRefractionIndex = -1;
        double xDistance = 0;
        for(Map.Entry<Double, Double> entry : meanAtmoshpericRefractions.entrySet()) {
            if(pos.getZ() < entry.getKey()) {
                continue;
            }
            if(previousRefractionIndex > 0) {
                incidenceAngleSin = previousRefractionIndex * incidenceAngleSin / entry.getValue();
            }
            xDistance += (pos.getZ() - entry.getKey()) * FastMath.tan(incidenceAngleSin);

            if(altitude > entry.getKey()) {
                break;
            }
            previousRefractionIndex = entry.getValue();
        }

        NormalizedGeodeticPoint geodeticPoint = tile.cellIntersection(pos, los, 0, 0);

        return pos.getX() + xDistance;
    }
}
