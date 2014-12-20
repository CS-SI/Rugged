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

    /** Create builder.
     * @param rawLOS raw fixed lines-of-sight
     */
    public LOSBuilder(final List<Vector3D> rawLOS) {
        this.rawLOS = rawLOS;
    }

    /** Build a list of transformed lines-of-sight.
     * @return list of transformed lines of sight
     */
    public List<TimeDependentLOS> build() {
        final List<TimeDependentLOS> los = new ArrayList<TimeDependentLOS>(rawLOS.size());
        for (final Vector3D raw : rawLOS) {
            los.add(new FixedLOS(raw));
        }
        return los;
    }

}
