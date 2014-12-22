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

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

/** Interface for lines-of-sight tranforms that do not depend on time.
 * @author Luc Maisonobe
 * @see LOSBuilder
 */
interface TimeIndependentLOSTransform {

    /** Transform a line-of-sight.
     * @param i los pixel index
     * @param los line-of-sight to transform
     * @return transformed line-of-sight
     */
    Vector3D transformLOS(int i, Vector3D los);

}
