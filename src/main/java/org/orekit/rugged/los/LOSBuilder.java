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
    public final List<Vector3D> rawLOS;

    /** Transforms to be applied. */
    private final List<LOSTransform> transforms;

    /** Create builder.
     * @param rawLOS raw fixed lines-of-sight
     */
    public LOSBuilder(final List<Vector3D> rawLOS) {
        this.rawLOS = rawLOS;
        this.transforms = new ArrayList<LOSTransform>();
    }

    /** Add a transform to be applied after the already registered transforms.
     * @param transform transform to be applied to the lines-of-sight
     */
    public void addTransform(final LOSTransform transform) {
        transforms.add(transform);
        
    }

    /** Build a list of transformed lines-of-sight.
     * @return list of transformed lines of sight
     */
    public List<TimeDependentLOS> build() {

        // copy the current transforms set, to ensure immutability
        // of the built list, in case addTransform is called again after build
        final List<LOSTransform> copy = new ArrayList<LOSTransform>(transforms);
        final List<TimeDependentLOS> transformed = new ArrayList<TimeDependentLOS>(rawLOS.size());
        for (int i = 0; i < rawLOS.size(); ++i) {
            transformed.add(new TransformsSequenceLOS(i, rawLOS.get(i), copy));
        }

        return transformed;

    }

    /** Implement time-dependent LOS by applying all registered transforms. */
    private static class TransformsSequenceLOS implements TimeDependentLOS {

        /** LOS index. */
        private final int index;

        /** Raw direction. */
        private final Vector3D raw;

        /** Transforms to be applied. */
        private final List<LOSTransform> transforms;

        /** Simple constructor.
         * @param index los index
         * @param raw raw direction
         * @param transforms transforms to apply
         */
        public TransformsSequenceLOS(final int index, final Vector3D raw, final List<LOSTransform> transforms) {
            this.index      = index;
            this.raw        = raw;
            this.transforms = transforms;
        }

        /** {@inheritDoc} */
        @Override
        public Vector3D getLOS(final AbsoluteDate date) {
            Vector3D los = raw;
            for (final LOSTransform transform : transforms) {
                los = transform.transformLOS(index, los, date);
            }
            return los.normalize();
        }
        
    }

}
