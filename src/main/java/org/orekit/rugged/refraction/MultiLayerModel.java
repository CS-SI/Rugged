/* Copyright 2013-2018 CS Systèmes d'Information
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

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.intersection.IntersectionAlgorithm;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.utils.ExtendedEllipsoid;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;

/**
 * Atmospheric refraction model based on multiple layers with associated refractive index.
 * @author Sergio Esteves
 * @author Luc Maisonobe
 * @author Guylaine Prat
 * @since 2.0
 */
public class MultiLayerModel extends AtmosphericRefraction {

    /** Observed body ellipsoid. */
    private final ExtendedEllipsoid ellipsoid;

    /** Constant refraction layers. */
    private final List<ConstantRefractionLayer> refractionLayers;

    /** Atmosphere lowest altitude (m). */
    private final double atmosphereLowestAltitude;

    /** Simple constructor.
     * <p>
     * This model uses a built-in set of layers.
     * </p>
     * @param ellipsoid the ellipsoid to be used.
     */
    public MultiLayerModel(final ExtendedEllipsoid ellipsoid) {

        super();

        this.ellipsoid = ellipsoid;

        this.refractionLayers = new ArrayList<ConstantRefractionLayer>(15);
        this.refractionLayers.add(new ConstantRefractionLayer(100000.00, 1.000000));
        this.refractionLayers.add(new ConstantRefractionLayer( 50000.00, 1.000000));
        this.refractionLayers.add(new ConstantRefractionLayer( 40000.00, 1.000001));
        this.refractionLayers.add(new ConstantRefractionLayer( 30000.00, 1.000004));
        this.refractionLayers.add(new ConstantRefractionLayer( 23000.00, 1.000012));
        this.refractionLayers.add(new ConstantRefractionLayer( 18000.00, 1.000028));
        this.refractionLayers.add(new ConstantRefractionLayer( 14000.00, 1.000052));
        this.refractionLayers.add(new ConstantRefractionLayer( 11000.00, 1.000083));
        this.refractionLayers.add(new ConstantRefractionLayer(  9000.00, 1.000106));
        this.refractionLayers.add(new ConstantRefractionLayer(  7000.00, 1.000134));
        this.refractionLayers.add(new ConstantRefractionLayer(  5000.00, 1.000167));
        this.refractionLayers.add(new ConstantRefractionLayer(  3000.00, 1.000206));
        this.refractionLayers.add(new ConstantRefractionLayer(  1000.00, 1.000252));
        this.refractionLayers.add(new ConstantRefractionLayer(     0.00, 1.000278));
        this.refractionLayers.add(new ConstantRefractionLayer( -1000.00, 1.000306));

        // get the lowest altitude of the atmospheric model
        this.atmosphereLowestAltitude = refractionLayers.get(refractionLayers.size() - 1).getLowestAltitude();
    }

    /** Simple constructor.
     * @param ellipsoid the ellipsoid to be used.
     * @param refractionLayers the refraction layers to be used with this model (layers can be in any order).
     */
    public MultiLayerModel(final ExtendedEllipsoid ellipsoid, final List<ConstantRefractionLayer> refractionLayers) {

        super();
        // at this stage no optimization is set up: no optimization grid is defined

        this.ellipsoid = ellipsoid;
        this.refractionLayers = new ArrayList<>(refractionLayers);

        // sort the layers from the highest (index = 0) to the lowest (index = size - 1)
        Collections.sort(this.refractionLayers,
            (l1, l2) -> Double.compare(l2.getLowestAltitude(), l1.getLowestAltitude()));

        // get the lowest altitude of the model
        atmosphereLowestAltitude = this.refractionLayers.get(this.refractionLayers.size() - 1).getLowestAltitude();
    }

    /** Compute the (position, LOS) of the intersection with the lowest atmospheric layer.
     * @param satPos satellite position, in body frame
     * @param satLos satellite line of sight, in body frame
     * @param rawIntersection intersection point without refraction correction
     * @return the intersection position and LOS with the lowest atmospheric layer
     * @since 3.0
     */
    private IntersectionLOS computeToLowestAtmosphereLayer(
                            final Vector3D satPos, final Vector3D satLos,
                            final NormalizedGeodeticPoint rawIntersection) {

        if (rawIntersection.getAltitude() < atmosphereLowestAltitude) {
            throw new RuggedException(RuggedMessages.NO_LAYER_DATA, rawIntersection.getAltitude(),
                                      atmosphereLowestAltitude);
        }

        Vector3D pos = satPos;
        Vector3D los = satLos.normalize();

        // Compute the intersection point with the lowest layer of atmosphere
        double n1 = -1;
        GeodeticPoint gp = ellipsoid.transform(satPos, ellipsoid.getBodyFrame(), null);

        // Perform the exact computation (no optimization)
        // TBN: the refractionLayers is ordered from the highest to the lowest
        for (ConstantRefractionLayer refractionLayer : refractionLayers) {

            if (refractionLayer.getLowestAltitude() > gp.getAltitude()) {
                continue;
            }

            final double n2 = refractionLayer.getRefractiveIndex();

            if (n1 > 0) {

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
                //   (5) α  = n₁/n₂
                // the α coefficient is independent from the incidence angle,
                // it depends only on the ratio of refractive indices!
                //
                // back to equation (4) and using again the fact θ₂ is between 0 and π/2, we can now write
                //       β = α cos θ₁ - cos θ₂
                //         = n₁/n₂ cos θ₁ - cos θ₂
                //         = n₁/n₂ cos θ₁ - √(1 - sin² θ₂)
                //         = n₁/n₂ cos θ₁ - √(1 - (n₁/n₂)² sin² θ₁)
                //         = n₁/n₂ cos θ₁ - √(1 - (n₁/n₂)² (1 - cos² θ₁))
                //         = n₁/n₂ cos θ₁ - √(1 - (n₁/n₂)² + (n₁/n₂)² cos² θ₁)
                //   (6) β = -k - √(k² - ζ)
                // where ζ = (n₁/n₂)² - 1 and k = n₁/n₂ u·z, which is negative, and close to -1 for
                // nadir observations. As we expect atmosphere models to have small transitions between
                // layers, we have to handle accurately the case where n₁/n₂ ≈ 1 so ζ ≈ 0. In this case,
                // a cancellation occurs inside the square root: √(k² - ζ) ≈ √k² ≈ -k (because k is negative).
                // So β ≈ -k + k ≈ 0 and another cancellation occurs, outside of the square root.
                // This means that despite equation (6) is mathematically correct, it is prone to numerical
                // errors when consecutive layers have close refractive indices. A better equivalent
                // expression is needed. The fact β is close to 0 in this case was expected because
                // equation (1) reads v = α u + β z, and α = n₁/n₂, so when n₁/n₂ ≈ 1, we have
                // α ≈ 1 and β ≈ 0, so v ≈ u: when two layers have similar refractive indices, the
                // propagation direction is almost unchanged.
                //
                // The first step for the accurate computation of β is to compute ζ = (n₁/n₂)² - 1
                // accurately and avoid a cancellation just after a division (which is the least accurate
                // of the four operations) and a squaring. We will simply use:
                //   ζ = (n₁/n₂)² - 1
                //     = (n₁ - n₂) (n₁ + n₂) / n₂²
                // The cancellation is still there, but it occurs in the subtraction n₁ - n₂, which are
                // the most accurate values we can get.
                // The second step for the accurate computation of β is to rewrite equation (6)
                // by both multiplying and dividing by the dual factor -k + √(k² - ζ):
                //     β = -k - √(k² - ζ)
                //       = (-k - √(k² - ζ)) * (-k + √(k² - ζ)) / (-k + √(k² - ζ))
                //       = (k² - (k² - ζ)) / (-k + √(k² - ζ))
                // (7) β = ζ / (-k + √(k² - ζ))
                // expression (7) is more stable numerically than expression (6), because when ζ ≈ 0
                // its denominator is about -2k, there are no cancellation anymore after the square root.
                // β is computed with the same accuracy as ζ
                final double alpha = n1 / n2;
                final double k     = alpha * Vector3D.dotProduct(los, gp.getZenith());
                final double zeta  = (n1 - n2) * (n1 + n2) / (n2 * n2);
                final double beta  = zeta / (FastMath.sqrt(k * k - zeta) - k);
                los = new Vector3D(alpha, los, beta, gp.getZenith());
            }

            // In case the altitude of the intersection without atmospheric refraction
            // is above the lowest altitude of the atmosphere: stop the search
            if (rawIntersection.getAltitude() > refractionLayer.getLowestAltitude()) {
                break;
            }

            // Get for the intersection point: the position and the LOS
            pos = ellipsoid.pointAtAltitude(pos, los, refractionLayer.getLowestAltitude());
            gp  = ellipsoid.transform(pos, ellipsoid.getBodyFrame(), null);

            n1 = n2;
        }
        return new IntersectionLOS(pos, los);
    }

    /** {@inheritDoc} */
    @Override
    public NormalizedGeodeticPoint applyCorrection(final Vector3D satPos, final Vector3D satLos,
                                                   final NormalizedGeodeticPoint rawIntersection,
                                                   final IntersectionAlgorithm algorithm)
                                                   {

        final IntersectionLOS intersectionLOS = computeToLowestAtmosphereLayer(satPos, satLos, rawIntersection);
        final Vector3D pos = intersectionLOS.getIntersectionPos();
        final Vector3D los = intersectionLOS.getIntersectionLos();

        // at this stage the pos belongs to the lowest atmospheric layer.
        // We can compute the intersection of line of sight (los) with Digital Elevation Model
        // as usual (without atmospheric refraction).
        return algorithm.refineIntersection(ellipsoid, pos, los, rawIntersection);
    }

    /** {@inheritDoc} */
    @Override
    public NormalizedGeodeticPoint applyCorrection(final LineSensor lineSensor, final SensorPixel sensorPixel,
                                                   final Vector3D satPos, final Vector3D satLos,
                                                   final NormalizedGeodeticPoint rawIntersection,
                                                   final IntersectionAlgorithm algorithm) {

        // TODO to be done
        throw new RuggedException(RuggedMessages.UNINITIALIZED_CONTEXT, "Atmospheric optimization for direct loc");
    }

} // end of class MultiLayerModel

/** Container for the (position, LOS) of the intersection with the lowest atmospheric layer.
 * @author Guylaine Prat
 * @since 3.0
 */
class IntersectionLOS {

    /** Position of the intersection with the lowest atmospheric layer. */
    private Vector3D intersectionPos;
    /** LOS of the intersection with the lowest atmospheric layer. */
    private Vector3D intersectionLos;

    /** Default constructor.
     * @param intersectionPos position of the intersection
     * @param intersectionLos los of the intersection
     */
    IntersectionLOS(final Vector3D intersectionPos, final Vector3D intersectionLos) {
        this.intersectionPos = intersectionPos;
        this.intersectionLos = intersectionLos;
    }
    public Vector3D getIntersectionPos() {
        return intersectionPos;
    }

    public Vector3D getIntersectionLos() {
        return intersectionLos;
    }
}
