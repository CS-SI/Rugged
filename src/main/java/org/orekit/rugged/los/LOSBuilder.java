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
package org.orekit.rugged.los;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.orekit.time.AbsoluteDate;

/** Builder for lines-of-sight list.
 * <p>
 * This builder aims at creating lines-of-sight directions which are
 * the result of several transforms applied to an initial list of raw
 * directions. It therefore allows to take into account the optical
 * path due to mirrors and the alignments of sensors frames with respect
 * to a spacecraft.
 * </p>
 * @see TimeDependentLOS
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
     */
    public void addTransform(final TimeIndependentLOSTransform transform) {
        transforms.add(new TransformAdapter(transform));
    }

    /** Add a transform to be applied after the already registered transforms.
     * @param transform transform to be applied to the lines-of-sight
     */
    public void addTransform(final LOSTransform transform) {
        transforms.add(transform);
        timeIndependent = false;
    }

    /** Build a list of transformed lines-of-sight.
     * @return list of transformed lines of sight
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

        /** Get the underlying transform.
         * @return underlying time-independent transform
         */
        public TimeIndependentLOSTransform getTransform() {
            return transform;
        }

        /** {@inheritDoc} */
        @Override
        public Vector3D transformLOS(final int i, final Vector3D los, final AbsoluteDate date) {
            return transform.transformLOS(i, los);
        }

    }

    /** Implement time-independent LOS by applying all registered transforms at construction. */
    private static class FixedLOS implements TimeDependentLOS {

        /** Fixed direction for los. */
        private final Vector3D[]  los;

        /** Simple constructor.
         * @param raw raw directions
         * @param transforms transforms to apply (must be time-independent!)
         */
        public FixedLOS(final List<Vector3D> raw, final List<LOSTransform> transforms) {

            los = new Vector3D[raw.size()];

            // apply transforms only once
            for (int i = 0; i < raw.size(); ++i) {
                Vector3D v = raw.get(i);
                for (final LOSTransform transform : transforms) {
                    v = ((TransformAdapter) transform).getTransform().transformLOS(i, v);
                }
                los[i] = v.normalize();
            }

        }

        /** {@inheritDoc} */
        public int getNbPixels() {
            return los.length;
        }

        /** {@inheritDoc} */
        public Vector3D getLOS(final int index, final AbsoluteDate date) {
            return los[index];
        }

    }

    /** Implement time-dependent LOS by applying all registered transforms at runtime. */
    private static class TransformsSequenceLOS implements TimeDependentLOS {

        /** Raw direction. */
        private final Vector3D[] raw;

        /** Transforms to be applied. */
        private final LOSTransform[] transforms;

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
            for (int i = 0; i < transforms.size(); ++i) {
                this.transforms[i] = transforms.get(i);
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

    }

}
