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
package org.orekit.rugged.refraction;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
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
 * Atmospheric refraction model based on multiple layers with associated refractive index.
 * @author Sergio Esteves, Luc Maisonobe
 * @since 2.0
 */
public class MultiLayerModel implements AtmosphericRefraction {

    /** Observed body ellipsoid. */
    private final ExtendedEllipsoid ellipsoid;

    /** Constant refraction layers. */
    private final List<ConstantRefractionLayer> refractionLayers;

    /** Atmosphere lowest altitude. */
    private final double atmosphereLowestAltitude;

    /** Simple constructor.
     * <p>
     * This model uses a built-in set of layers.
     * </p>
     * @param ellipsoid the ellipsoid to be used.
     */
    public MultiLayerModel(final ExtendedEllipsoid ellipsoid) {
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

    /** Simple constructor.
     * @param ellipsoid the ellipsoid to be used.
     * @param refractionLayers the refraction layers to be used with this model (layers can be in any order).
     */
    public MultiLayerModel(final ExtendedEllipsoid ellipsoid, final List<ConstantRefractionLayer> refractionLayers) {
        this.ellipsoid = ellipsoid;
        this.refractionLayers = new ArrayList<>(refractionLayers);
        Collections.sort(this.refractionLayers,
                         (l1, l2) -> Double.compare(l2.getLowestAltitude(), l1.getLowestAltitude()));
        atmosphereLowestAltitude = this.refractionLayers.get(this.refractionLayers.size() - 1).getLowestAltitude();
    }

    /** {@inheritDoc} */
    @Override
    public NormalizedGeodeticPoint applyCorrection(final Vector3D satPos, final Vector3D satLos,
                                                   final NormalizedGeodeticPoint rawIntersection,
                                                   final IntersectionAlgorithm algorithm)
        throws RuggedException {

        try {
            if (rawIntersection.getAltitude() < atmosphereLowestAltitude) {
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

                    // when we get here, we have already performed one iteration in the loop
                    // so gp is the los intersection with the layers interface (it was a
                    // point on ground at loop initialization, but is overridden at each iteration)

                    // get new los by applying Snell's law at atmosphere layers interfaces
                    // we avoid computing sequences of inverse-trigo/trigo/inverse-trigo functions
                    // we just use linear algebra and square roots, it is faster and more accurate

                    // at interface crossing, the interface normal is z, the local zenith direction
                    // the ray direction (i.e. los) is u in the upper layer and v in the lower layer
                    // v is in the (u, zenith) plane, so we can say
                    //  (1) v = α u + β z
                    // with α>0 as u and v are roughly in the same direction as the ray is slightly bent

                    // let θ₁ be the los incidence angle at interface crossing
                    // θ₁ = π - angle(u, zenith) is between 0 and π/2 for a downwards observation
                    // let θ₂ be the exit angle at interface crossing
                    // from Snell's law, we have n₁ sin θ₁ = n₂ sin θ₂ and θ₂ is also between 0 and π/2
                    // we have:
                    //   (2) u·z = -cos θ₁
                    //   (3) v·z = -cos θ₂
                    // combining equations (1), (2) and (3) and remembering z·z = 1 as z is normalized , we get
                    //   (4) β = α cos θ₁ - cos θ₂
                    // with all the expressions above, we can rewrite the fact v is normalized:
                    //       1 = v·v
                    //         = α² u·u + 2αβ u·z + β² z·z
                    //         = α² - 2αβ cos θ₁ + β²
                    //         = α² - 2α² cos² θ₁ + 2 α cos θ₁ cos θ₂ + α² cos² θ₁ - 2 α cos θ₁ cos θ₂ + cos² θ₂
                    //         = α²(1 - cos² θ₁) + cos² θ₂
                    // hence α² = (1 - cos² θ₂)/(1 - cos² θ₁)
                    //          = sin² θ₂ / sin² θ₁
                    // as α is positive, and both θ₁ and θ₂ are between 0 and π/2, we finally get
                    //       α  = sin θ₂ / sin θ₁
                    //   (5) α  = n₁ / n₂
                    // the α coefficient is independent from the incidence angle,
                    // it depends only on the ratio of refractive indices!
                    //
                    // back to equation (4) and using again the fact θ₂ is between 0 and π/2, we can now write
                    //       β = α cos θ₁ - cos θ₂
                    //         = n₁ / n₂ cos θ₁ - cos θ₂
                    //         = n₁ / n₂ cos θ₁ - √(1 - sin² θ₂)
                    //         = n₁ / n₂ cos θ₁ - √(1 - (n₁ / n₂)² sin² θ₁)
                    //         = n₁ / n₂ cos θ₁ - √(1 - (n₁ / n₂)² (1 - cos² θ₁))
                    //         = n₁ / n₂ cos θ₁ - √(1 + (n₁ / n₂)² cos² θ₁ - (n₁ / n₂)²)
                    //   (6) β = -k - √(1 + k² - (n₁ / n₂)²)
                    // where k = n₁ / n₂ u·z
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
                gp  = ellipsoid.transform(pos, ellipsoid.getBodyFrame(), null);

                previousRefractiveIndex = refractionLayer.getRefractiveIndex();

            }

            return algorithm.refineIntersection(ellipsoid, pos, los, rawIntersection);

        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
    }
}
