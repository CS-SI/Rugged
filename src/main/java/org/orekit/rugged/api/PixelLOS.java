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

/** Container for pixels line of sight.
 * <p>
 * Instances of this class are guaranteed to be immutable.
 * </p>
 * @author Luc Maisonobe
 */
public class PixelLOS implements Serializable {

    /** Serializable UID. */
    private static final long serialVersionUID = 20140311L;

    /** Position along x axis(m). */
    private final double px;

    /** Position along y axis(m). */
    private final double py;

    /** Position along z axis(m). */
    private final double pz;

    /** Line of sight direction along x axis. */
    private final double dx;

    /** Line of sight direction along x axis. */
    private final double dy;

    /** Line of sight direction along x axis. */
    private final double dz;

    /**
     * Build a new instance.
     *
     * @param px position along x axis (m)
     * @param py position along y axis (m)
     * @param pz position along z axis (m)
     * @param dx line of sight direction along x axis
     * @param dy line of sight direction along y axis
     * @param dz line of sight direction along z axis
     */
    public PixelLOS(final double px, final double py, final double pz,
                    final double dx, final double dy, final double dz) {
        this.px   = px;
        this.py   = py;
        this.pz   = pz;
        this.dx   = dx;
        this.dy   = dy;
        this.dz   = dz;
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

    /** Get the line of sight direction along x axis.
     * @return line of sight direction along x axis
     */
    public double getDx() {
        return dx;
    }

    /** Get the line of sight direction along y axis.
     * @return line of sight direction along y axis
     */
    public double getDy() {
        return dy;
    }

    /** Get the line of sight direction along z axis.
     * @return line of sight direction along z axis
     */
    public double getDz() {
        return dz;
    }

}
