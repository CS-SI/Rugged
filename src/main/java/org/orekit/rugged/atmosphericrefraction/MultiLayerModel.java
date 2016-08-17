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
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.frames.FramesFactory;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.intersection.IntersectionAlgorithm;
import org.orekit.rugged.intersection.duvenhage.MinMaxTreeTile;
import org.orekit.rugged.intersection.duvenhage.MinMaxTreeTileFactory;
import org.orekit.rugged.raster.Tile;
import org.orekit.rugged.raster.TilesCache;
import org.orekit.rugged.utils.ExtendedEllipsoid;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * Multi layer model for atmospheric refraction.
 * @author Sergio Esteves
 */
public class MultiLayerModel implements AtmosphericRefraction {

    /** Observed body ellipsoid. */
    private final ExtendedEllipsoid ellipsoid;

    /** Constant refraction layers */
    private final List<ConstantRefractionLayer> refractionLayers;

    public MultiLayerModel(final ExtendedEllipsoid ellipsoid)
            throws OrekitException {
        this.ellipsoid = ellipsoid;

        refractionLayers = new ArrayList<ConstantRefractionLayer>(15);
        refractionLayers.add(new ConstantRefractionLayer(100000.00, 1.000000));
        refractionLayers.add(new ConstantRefractionLayer( 50000.00, 1.000000));
        refractionLayers.add(new ConstantRefractionLayer( 40000.00, 1.000001));
        refractionLayers.add(new ConstantRefractionLayer( 30000.00, 1.000004));
        refractionLayers.add(new ConstantRefractionLayer( 23000.00, 1.000012));
        refractionLayers.add(new ConstantRefractionLayer( 18000.00, 1.000028));
        refractionLayers.add(new ConstantRefractionLayer( 14000.00, 1.000052));
        refractionLayers.add(new ConstantRefractionLayer( 11000.00, 1.000083));
        refractionLayers.add(new ConstantRefractionLayer(  9000.00, 1.000106));
        refractionLayers.add(new ConstantRefractionLayer(  7000.00, 1.000134));
        refractionLayers.add(new ConstantRefractionLayer(  5000.00, 1.000167));
        refractionLayers.add(new ConstantRefractionLayer(  3000.00, 1.000206));
        refractionLayers.add(new ConstantRefractionLayer(  1000.00, 1.000252));
        refractionLayers.add(new ConstantRefractionLayer(     0.00, 1.000278));
        refractionLayers.add(new ConstantRefractionLayer( -1000.00, 1.000306));
    }

    public MultiLayerModel(final ExtendedEllipsoid ellipsoid, final List<ConstantRefractionLayer> refractionLayers)
            throws OrekitException {
        this.ellipsoid = ellipsoid;
        // TODO guarantee that list is already ordered by altitude?
        this.refractionLayers = refractionLayers;
    }

    @Override
    public NormalizedGeodeticPoint applyCorrection(final Vector3D satPos, final Vector3D satLos,
                                                   final NormalizedGeodeticPoint rawIntersection,
                                                   final IntersectionAlgorithm algorithm)
            throws RuggedException {

        try {

            Vector3D pos = satPos;
            Vector3D los = satLos;
            Vector3D zenith = null;
            double previousRefractionIndex = -1;
            GeodeticPoint gp = ellipsoid.transform(satPos, ellipsoid.getBodyFrame(), null);

            for(ConstantRefractionLayer refractionLayer : refractionLayers) {

                if(refractionLayer.getLowestAltitude() > gp.getAltitude()) {
                    continue;
                }

                if (previousRefractionIndex > 0) {

                    // get new los
                    final double theta1 = Vector3D.angle(los, zenith);
                    final double theta2 = FastMath.asin(previousRefractionIndex * FastMath.sin(theta1) /
                            refractionLayer.getRefractionIndex());

                    final double cosTheta1     = FastMath.cos(theta1);
                    final double cosTheta2     = FastMath.cos(theta2);

                    final double a = FastMath.sqrt((1 - cosTheta2 * cosTheta2) / (1 - cosTheta1 * cosTheta1));
                    final double b = a * cosTheta1 - cosTheta2;
                    los = new Vector3D(a, los, b, zenith);
                }

                if (rawIntersection.getAltitude() > refractionLayer.getLowestAltitude()) {
                    break;
                }

                // get intersection point
                pos = ellipsoid.pointAtAltitude(pos, los, refractionLayer.getLowestAltitude());
                gp = ellipsoid.transform(pos, ellipsoid.getBodyFrame(), null);
                zenith = gp.getZenith();

                previousRefractionIndex = refractionLayer.getRefractionIndex();
            }

            final NormalizedGeodeticPoint newGeodeticPoint  =
                    algorithm.refineIntersection(ellipsoid, pos, los, rawIntersection);

            return newGeodeticPoint;

        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
    }
}

class ConstantRefractionLayer {
    private double lowestAltitude;
    private double refractionIndex;

    public ConstantRefractionLayer(double lowestAltitude, double refractionIndex) {
        this.lowestAltitude = lowestAltitude;
        this.refractionIndex = refractionIndex;
    }

    public double getLowestAltitude() {
        return lowestAltitude;
    }

    public double getRefractionIndex() {
        return refractionIndex;
    }
}
