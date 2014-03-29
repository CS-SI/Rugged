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
package org.orekit.rugged.geotiff;


enum LinearUnits {

    METER(9001),
    FOOT(9002),
    FOOT_US_SURVEY(9003),
    FOOT_MODIFIED_AMERICAN(9004),
    FOOT_CLARKE(9005),
    FOOT_INDIAN(9006),
    LINK(9007),
    LINK_BENOIT(9008),
    LINK_SEARS(9009),
    CHAIN_BENOIT(9010),
    CHAIN_SEARS(9011),
    YARD_SEARS(9012),
    YARD_INDIAN(9013),
    FATHOM(9014),
    MILE_INTERNATIONAL_NAUTICAL(9015);

    /** Units ID. */
    private final int id;

    /** Simple constructor.
     * @param id key id
     */
    private LinearUnits(final int id) {
        this.id   = id;
    }

    /** Get the units corresponding to an id.
     * @param id type id
     * @return the units corresponding to the id
     * @exception IllegalArgumentException if the id does not correspond to known units
     */
    public static LinearUnits getUnits(final int id) {
        for (LinearUnits units : values()) {
            if (units.id == id) {
                return units;
            }
        }
        throw new IllegalArgumentException();
    }

}
