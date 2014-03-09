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
package org.orekit.rugged.api;

import java.io.Serializable;

/** Container for satellite position and velocity.
 * <p>
 * Instances of this class are guaranteed to be immutable.
 * </p>
 * @author Luc Maisonobe
 */
public class SatellitePV implements Serializable {

    /** Serializable UID. */
    private static final long serialVersionUID = 20140309L;

    /** UTC date. */
    private final String date;

    /** Position along x axis(m). */
    private final double px;

    /** Position along y axis(m). */
    private final double py;

    /** Position along z axis(m). */
    private final double pz;

    /** Velocity along x axis (m/s). */
    private final double vx;

    /** Velocity along y axis (m/s). */
    private final double vy;

    /** Velocity along z axis (m/s). */
    private final double vz;

    /**
     * Build a new instance.
     *
     * @param date UTC date
     * @param px position along x axis (m)
     * @param py position along y axis (m)
     * @param pz position along z axis (m)
     * @param vx velocity along x axis (m/s)
     * @param vy velocity along y axis (m/s)
     * @param vz velocity along z axis (m/s)
     */
    public SatellitePV(final String date,
                       final double px, final double py, final double pz,
                       final double vx, final double vy, final double vz) {
        this.date = date;
        this.px   = px;
        this.py   = py;
        this.pz   = pz;
        this.vx   = vx;
        this.vy   = vy;
        this.vz   = vz;
    }

    /** Get the UTC date.
     * @return utc date
     */
    public String getDate() {
        return date;
    }

    /** Get the position along x axis.
     * @return position along x axis
     */
    public double getPx() {
        return px;
    }

    /** Get the position along y axis.
     * @return position along y axis
     */
    public double getPy() {
        return py;
    }

    /** Get the position along z axis.
     * @return position along z axis
     */
    public double getPz() {
        return pz;
    }

    /** Get the velocity along x axis.
     * @return velocity along x axis
     */
    public double getVx() {
        return vx;
    }

    /** Get the velocity along y axis.
     * @return velocity along y axis
     */
    public double getVy() {
        return vy;
    }

    /** Get the velocity along z axis.
     * @return velocity along z axis
     */
    public double getVz() {
        return vz;
    }

}
