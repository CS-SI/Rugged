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
package org.orekit.rugged.los;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Stream;

import org.hipparchus.analysis.differentiation.Derivative;
import org.hipparchus.geometry.euclidean.threed.FieldVector3D;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.orekit.rugged.utils.DerivativeGenerator;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.ParameterDriver;
import org.orekit.utils.ParameterObserver;
import org.orekit.utils.TimeSpanMap;

/** Builder for lines-of-sight list.
 * <p>
 * This class implements the <em>builder pattern</em> to create {@link TimeDependentLOS} instances.
 * It does so by using a <em>fluent API</em> in order to clarify reading and allow
 * later extensions with new configuration parameters.
 * </p>
 * <p>
 * This builder aims at creating lines-of-sight directions which are
 * the result of several transforms applied to an initial list of raw
 * directions. It therefore allows to take into account the optical
 * path due to mirrors and the alignments of sensors frames with respect
 * to a spacecraft.
 * </p>
 * @see TimeDependentLOS
 * @see <a href="https://en.wikipedia.org/wiki/Builder_pattern">Builder pattern (wikipedia)</a>
 * @see <a href="https://en.wikipedia.org/wiki/Fluent_interface">Fluent interface (wikipedia)</a>
 * @author Luc Maisonobe
 */
public class LOSBuilder {

    /** Raw fixed line-of-sights. */
    private final List<Vector3D> rawLOS;

    /** Transforms to be applied. */
    private final List<LOSTransform> transforms;

    /** Flag for time-independent only transforms. */
    private boolean timeIndependent;

    /** Create builder.
     * @param rawLOS raw fixed lines-of-sight
     */
    public LOSBuilder(final List<Vector3D> rawLOS) {
        this.rawLOS          = rawLOS;
        this.transforms      = new ArrayList<>();
        this.timeIndependent = true;
    }

    /** Add a transform to be applied after the already registered transforms.
     * @param transform transform to be applied to the lines-of-sight
     * @return the builder instance
     */
    public LOSBuilder addTransform(final TimeIndependentLOSTransform transform) {
        transforms.add(new TransformAdapter(transform));
        return this;
    }

    /** Add a transform to be applied after the already registered transforms.
     * @param transform transform to be applied to the lines-of-sight
     * @return the builder instance
     */
    public LOSBuilder addTransform(final LOSTransform transform) {
        transforms.add(transform);
        timeIndependent = false;
        return this;
    }

    /** Build a lines-of-sight provider.
     * @return lines-of-sight provider
     */
    public TimeDependentLOS build() {

        if (timeIndependent) {
            // fast implementation for time-independent lines-of-sight
            return new FixedLOS(rawLOS, transforms);
        } else {
            // regular implementation, for time-dependent lines-of-sight
            return new TransformsSequenceLOS(rawLOS, transforms);
        }

    }

    /** Adapter from time-independent transform to time-dependent transform. */
    private static class TransformAdapter implements LOSTransform {

        /** Underlying transform. */
        private final TimeIndependentLOSTransform transform;

        /** Simple constructor.
         * @param transform underlying time-independent transform
         */
        TransformAdapter(final TimeIndependentLOSTransform transform) {
            this.transform = transform;
        }

        /** {@inheritDoc} */
        @Override
        public Vector3D transformLOS(final int i, final Vector3D los, final AbsoluteDate date) {
            return transform.transformLOS(i, los);
        }

        /** {@inheritDoc} */
        @Override
        public <T extends Derivative<T>> FieldVector3D<T> transformLOS(final int i, final FieldVector3D<T> los,
                                                                       final AbsoluteDate date, final DerivativeGenerator<T> generator) {
            return transform.transformLOS(i, los, generator);
        }

        /** {@inheritDoc} */
        @Override
        public Stream<ParameterDriver> getParametersDrivers() {
            return transform.getParametersDrivers();
        }

    }

    /** Implement time-independent LOS by recomputing directions by applying all transforms each time. */
    private static class TransformsSequenceLOS implements TimeDependentLOS {

        /** Raw direction. */
        private final Vector3D[] raw;

        /** Transforms to be applied. */
        private final List<LOSTransform> transforms;

        /** Simple constructor.
         * @param raw raw directions
         * @param transforms transforms to apply
         */
        TransformsSequenceLOS(final List<Vector3D> raw, final List<LOSTransform> transforms) {

            // copy the lists, to ensure immutability of the built object,
            // in case addTransform is called again after build
            // or the raw LOS list is changed by caller
            this.raw = new Vector3D[raw.size()];
            for (int i = 0; i < raw.size(); ++i) {
                this.raw[i] = raw.get(i);
            }

            this.transforms = new ArrayList<>(transforms);

        }

        /** {@inheritDoc} */
        public int getNbPixels() {
            return raw.length;
        }

        /** {@inheritDoc} */
        @Override
        public Vector3D getLOS(final int index, final AbsoluteDate date) {
            Vector3D los = raw[index];
            for (final LOSTransform transform : transforms) {
                los = transform.transformLOS(index, los, date);
            }
            return los.normalize();
        }

        /** {@inheritDoc} */
        @Override
        public <T extends Derivative<T>> FieldVector3D<T> getLOSDerivatives(final int index, final AbsoluteDate date,
                                                                            final DerivativeGenerator<T> generator) {

            // the raw line of sights are considered to be constant
            FieldVector3D<T> los = new FieldVector3D<>(generator.constant(raw[index].getX()),
                                                       generator.constant(raw[index].getY()),
                                                       generator.constant(raw[index].getZ()));

            // apply the transforms, which depend on parameters and hence may introduce non-zero derivatives
            for (final LOSTransform transform : transforms) {
                los = transform.transformLOS(index, los, date, generator);
            }

            return los.normalize();

        }

        @Override
        public Stream<ParameterDriver> getParametersDrivers() {
            Stream<ParameterDriver> drivers = Stream.<ParameterDriver>empty();
            for (final LOSTransform transform : transforms) {
                drivers = Stream.concat(drivers, transform.getParametersDrivers());
            }
            return drivers;
        }

    }

    /** Implement time-independent LOS by computing directions only when parameters are changed. */
    private static class FixedLOS extends TransformsSequenceLOS {

        /** transformed direction for los. */
        private final Vector3D[] transformed;

        /** Simple constructor.
         * @param raw raw directions
         * @param transforms transforms to apply (must be time-independent!)
         */
        FixedLOS(final List<Vector3D> raw, final List<LOSTransform> transforms) {

            super(raw, transforms);
            transformed = new Vector3D[raw.size()];

            // we will reset the transforms to null when parameters are changed
            final ParameterObserver resettingObserver = new ParameterObserver() {
                /** {@inheritDoc} */
                @Override
                public void valueChanged(final double previousValue, final ParameterDriver driver, final AbsoluteDate date) {
                    Arrays.fill(transformed, null);
                }

                /** {@inheritDoc} */
                @Override
                public void valueSpanMapChanged​(final TimeSpanMap<Double> previousValueSpanMap, final ParameterDriver driver) {
                    Arrays.fill(transformed, null);
                }
            };
            getParametersDrivers().forEach(driver -> {
                driver.addObserver(resettingObserver);
            });
        }

        /** {@inheritDoc} */
        @Override
        public Vector3D getLOS(final int index, final AbsoluteDate date) {
            if (transformed[index] == null) {
                // recompute the transformed los direction only if needed
                transformed[index] = super.getLOS(index, date);
            }
            return transformed[index];
        }

    }

}
