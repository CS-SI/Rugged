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
package org.orekit.rugged.raster;

/**
 * Enumerate for Earth hemispheres for tiles definition.
 * <p>
 * For Latitude: NORTH / SOUTH
 * <p>
 * For Longitude: WESTEXTREME / WEST / EAST / EASTEXTREME
 * @author Guylaine Prat
 * @since 4.0
 */
public enum EarthHemisphere {

    /** South hemisphere. */
   SOUTH,
   /** North hemisphere. */
   NORTH,
   /** Extreme West. */
   WESTEXTREME,
   /** West hemisphere. */
   WEST,
   /** East hemisphere. */
   EAST,
   /** Extreme East. */
   EASTEXTREME
}
