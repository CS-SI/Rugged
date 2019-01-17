/* Copyright 2013-2019 CS Systèmes d'Information
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

import org.orekit.rugged.intersection.duvenhage.MinMaxTreeTile;

public class ZipperTile extends MinMaxTreeTile {


    public ZipperTile() {
        
    }

    /** Create the geometry of the zipper tile from two tiles
     */
    public void createGeometry(final double zipperLatitudeMin, final double zipperLatitudeStep, final int zipperLatitudeRows,
                               final double zipperLongitudeMin, final double zipperLongitudeStep, final int zipperLongitudeColumns,
                               double[][] zipperElevations) {
        // Set the tile geometry
        setGeometry(zipperLatitudeMin, zipperLongitudeMin, zipperLatitudeStep, zipperLongitudeStep, zipperLatitudeRows, zipperLongitudeColumns);
        
        // Fill in the tile with the relevant elevations
        for (int iLat = 0; iLat < zipperLatitudeRows; iLat++) {
            for (int jLon = 0; jLon < zipperLongitudeColumns; jLon++) {
                setElevation(iLat, jLon, zipperElevations[iLat][jLon]);
            }
        }
    }
    
    /** {@inheritDoc} */
    @Override
    public void setGeometry(final double zipperLatitudeMin, final double zipperLongitudeMin,
                            final double zipperLatitudeStep, final double zipperLongitudeStep,
                            final int zipperLatitudeRows, final int zipperLongitudeColumns) {
        // Set the tile geometry
        super.setGeometry(zipperLatitudeMin, zipperLongitudeMin, zipperLatitudeStep, zipperLongitudeStep, zipperLatitudeRows, zipperLongitudeColumns);
    }
    
    /** {@inheritDoc} */
    @Override
    public void setElevation(final int latitudeIndex, final int longitudeIndex, final double elevation) {
        super.setElevation(latitudeIndex, longitudeIndex, elevation);
    }

}
