/* Copyright 2013-2015 CS Systèmes d'Information
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
package org.orekit.rugged.los;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.apache.commons.math3.analysis.differentiation.DerivativeStructure;
import org.apache.commons.math3.geometry.euclidean.threed.FieldVector3D;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.utils.ParametricModel;
import org.orekit.time.AbsoluteDate;

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

    /** Raw fixed ine-of-sights. */
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
        this.transforms      = new ArrayList<LOSTransform>();
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
        public TransformAdapter(final TimeIndependentLOSTransform transform) {
            this.transform = transform;
        }

        /** {@inheritDoc} */
        @Override
        public int getNbEstimatedParameters() {
            return transform.getNbEstimatedParameters();
        }

        /** {@inheritDoc} */
        @Override
        public void getEstimatedParameters(final double[] parameters, final int start, final int length)
            throws RuggedException {
            transform.getEstimatedParameters(parameters, start, length);
        }

        /** {@inheritDoc} */
        @Override
        public void setEstimatedParameters(final double[] parameters, final int start, final int length)
            throws RuggedException {
            transform.setEstimatedParameters(parameters, start, length);
        }

        /** {@inheritDoc} */
        @Override
        public Vector3D transformLOS(final int i, final Vector3D los, final AbsoluteDate date) {
            return transform.transformLOS(i, los);
        }

        /** {@inheritDoc} */
        @Override
        public FieldVector3D<DerivativeStructure> transformLOS(final int i, final FieldVector3D<DerivativeStructure> los,
                                                               final AbsoluteDate date) {
            return transform.transformLOS(i, los);
        }

    }

    /** Implement time-independent LOS by recomputing directions by applying all transforms each time. */
    private static class TransformsSequenceLOS implements ParametricModel, TimeDependentLOS {

        /** Raw direction. */
        private final Vector3D[] raw;

        /** Transforms to be applied. */
        private final LOSTransform[] transforms;

        /** Total number of estimated parameters. */
        private final int total;

        /** Simple constructor.
         * @param raw raw directions
         * @param transforms transforms to apply
         */
        public TransformsSequenceLOS(final List<Vector3D> raw, final List<LOSTransform> transforms) {

            // copy the lists, to ensure immutability of the built object,
            // in case addTransform is called again after build
            // or the raw LOS list is changed by caller
            this.raw = new Vector3D[raw.size()];
            for (int i = 0; i < raw.size(); ++i) {
                this.raw[i] = raw.get(i);
            }

            this.transforms = new LOSTransform[transforms.size()];
            int n = 0;
            for (int i = 0; i < transforms.size(); ++i) {
                final LOSTransform transform = transforms.get(i);
                this.transforms[i] = transform;
                n += transform.getNbEstimatedParameters();
            }
            this.total = n;

        }

        /** {@inheritDoc} */
        @Override
        public int getNbEstimatedParameters() {
            return total;
        }

        /** {@inheritDoc} */
        @Override
        public void getEstimatedParameters(final double[] parameters, final int start, final int length)
            throws RuggedException {

            // global check
            checkSlice(length);

            // retrieve parameters for all transforms
            int offset = 0;
            for (final ParametricModel model : transforms) {
                final int n = model.getNbEstimatedParameters();
                model.getEstimatedParameters(parameters, offset, n);
                offset += n;
            }

        }

        /** {@inheritDoc} */
        @Override
        public void setEstimatedParameters(final double[] parameters, final int start, final int length)
            throws RuggedException {

            // global check
            checkSlice(length);

            // set parameters for all transforms
            int offset = 0;
            for (final ParametricModel model : transforms) {
                final int n = model.getNbEstimatedParameters();
                model.setEstimatedParameters(parameters, offset, n);
                offset += n;
            }

        }

        /** Check the number of parameters of an array slice.
         * @param length number of elements in the array slice to consider
         * @exception RuggedException if the size of the slice does not match
         * the {@link #getNbEstimatedParameters() number of estimated parameters}
         */
        private void checkSlice(final int length) throws RuggedException {
            if (getNbEstimatedParameters() != length) {
                throw new RuggedException(RuggedMessages.ESTIMATED_PARAMETERS_NUMBER_MISMATCH,
                                          getNbEstimatedParameters(), length);
            }
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
        public FieldVector3D<DerivativeStructure> getLOS(final int index, final AbsoluteDate date,
                                                         final double[] parameters) {
            // non-adjustable LOS do not depend on any parameters
            final Vector3D los = getLOS(index, date);
            return new FieldVector3D<DerivativeStructure>(new DerivativeStructure(parameters.length, 1, los.getX()),
                                                          new DerivativeStructure(parameters.length, 1, los.getY()),
                                                          new DerivativeStructure(parameters.length, 1, los.getZ()));
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
        public FixedLOS(final List<Vector3D> raw, final List<LOSTransform> transforms) {
            super(raw, transforms);
            transformed = new Vector3D[raw.size()];
        }

        /** {@inheritDoc} */
        @Override
        public void setEstimatedParameters(final double[] parameters, final int start, final int length)
            throws RuggedException {

            // update the transforms
            super.setEstimatedParameters(parameters, start, length);

            // unset the directions, to ensure they get recomputed if needed
            Arrays.fill(transformed, null);

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
