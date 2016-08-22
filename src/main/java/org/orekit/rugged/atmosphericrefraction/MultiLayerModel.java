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
import org.orekit.errors.OrekitException;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.intersection.IntersectionAlgorithm;
import org.orekit.rugged.utils.ExtendedEllipsoid;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Multi layer model for atmospheric refraction.
 * @author Sergio Esteves, Luc Maisonobe
 */
public class MultiLayerModel implements AtmosphericRefraction {

    /** Observed body ellipsoid. */
    private final ExtendedEllipsoid ellipsoid;

    /** Constant refraction layers */
    private final List<ConstantRefractionLayer> refractionLayers;

    /** Atmosphere lowest altitude */
    private final double atmosphereLowestAltitude;

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

        atmosphereLowestAltitude = refractionLayers.get(refractionLayers.size() - 1).getLowestAltitude();
    }

    public MultiLayerModel(final ExtendedEllipsoid ellipsoid, final List<ConstantRefractionLayer> refractionLayers)
            throws OrekitException {
        this.ellipsoid = ellipsoid;
        this.refractionLayers = refractionLayers;
        Collections.sort(this.refractionLayers, Collections.<ConstantRefractionLayer>reverseOrder());
        atmosphereLowestAltitude = refractionLayers.get(refractionLayers.size() - 1).getLowestAltitude();
    }

    @Override
    public NormalizedGeodeticPoint applyCorrection(final Vector3D satPos, final Vector3D satLos,
                                                   final NormalizedGeodeticPoint rawIntersection,
                                                   final IntersectionAlgorithm algorithm)
            throws RuggedException {

        try {
            if(rawIntersection.getAltitude() < atmosphereLowestAltitude) {
                throw new RuggedException(RuggedMessages.NO_LAYER_DATA, rawIntersection.getAltitude(),
                        atmosphereLowestAltitude);
            }

            Vector3D pos = satPos;
            Vector3D los = satLos.normalize();
            double previousRefractiveIndex = -1;
            GeodeticPoint gp = ellipsoid.transform(satPos, ellipsoid.getBodyFrame(), null);

            for (ConstantRefractionLayer refractionLayer : refractionLayers) {

                if (refractionLayer.getLowestAltitude() > gp.getAltitude()) {
                    continue;
                }

                if (previousRefractiveIndex > 0) {

                    // get new los by applying Snell's law at atmosphere layers interfaces
                    // we avoid computing sequences of inverse-trigo/trigo/inverse-trigo functions
                    // we just use linear algebra and square roots, it is faster and more accurate
                    final double n1On2 = previousRefractiveIndex / refractionLayer.getRefractiveIndex();
                    final double k     = n1On2 * Vector3D.dotProduct(los, gp.getZenith());
                    los = new Vector3D(n1On2, los,
                                       -k - FastMath.sqrt(1 + k * k - n1On2 * n1On2), gp.getZenith());

                }

                if (rawIntersection.getAltitude() > refractionLayer.getLowestAltitude()) {
                    break;
                }

                // get intersection point
                pos = ellipsoid.pointAtAltitude(pos, los, refractionLayer.getLowestAltitude());
                gp = ellipsoid.transform(pos, ellipsoid.getBodyFrame(), null);

                previousRefractiveIndex = refractionLayer.getRefractiveIndex();
            }

            final NormalizedGeodeticPoint newGeodeticPoint  =
                    algorithm.refineIntersection(ellipsoid, pos, los, rawIntersection);

            return newGeodeticPoint;

        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
    }
}
