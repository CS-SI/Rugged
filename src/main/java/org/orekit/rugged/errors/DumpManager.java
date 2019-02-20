/* Copyright 2013-2019 CS Systèmes d'Information
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
package org.orekit.rugged.errors;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;

import org.orekit.bodies.GeodeticPoint;
import org.orekit.frames.Transform;
import org.orekit.rugged.api.AlgorithmId;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorMeanPlaneCrossing;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.raster.Tile;
import org.orekit.rugged.utils.ExtendedEllipsoid;
import org.orekit.rugged.utils.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;

/**
 * Class managing debug dumps.
 * <p>
 * <em>WARNING</em>: this class is public only for technical reasons,
 * it is not considered to belong to the public API of the library and should
 * not be called by user code. It is only intended to be called internally by
 * the Rugged library itself. This class may be changed or even removed at any
 * time, so user code should not rely on it.
 * </p>
 * @author Luc Maisonobe
 * @author Guylaine Prat
 */
public class DumpManager {

    /** Dump file (default initial value is null, i.e. nothing is dumped). */
    private static final ThreadLocal<Dump> DUMP = new ThreadLocal<Dump>();

    /** Boolean to check if the dump is suspended. */
    private static boolean isSuspended = false;

    /** Private constructor for utility class.
     */
    private DumpManager() {
        // by default, nothing is dumped
    }

    /** Activate debug dump.
     * @param file dump file
     */
    public static void activate(final File file) {
        if (isActive()) {
            throw new RuggedException(RuggedMessages.DEBUG_DUMP_ALREADY_ACTIVE);
        } else {
            try {
                DUMP.set(new Dump(new PrintWriter(file, "UTF-8")));
            } catch (IOException ioe) {
                throw new RuggedException(ioe, RuggedMessages.DEBUG_DUMP_ACTIVATION_ERROR,
                                          file.getAbsolutePath(), ioe.getLocalizedMessage());
            }
        }
    }

    /** Deactivate debug dump.
     */
    public static void deactivate() {
        if (isActive()) {
            DUMP.get().deactivate();
            DUMP.set(null);
        } else {
            throw new RuggedException(RuggedMessages.DEBUG_DUMP_NOT_ACTIVE);
        }
    }

    /** Suspend the dump.
     * In case the dump is already suspended, keep the previous status in order to
     * correctly deal the resume stage.
     * @return a flag to tell if the dump is already suspended (true; false otherwise)
     */
    public static Boolean suspend() {
        // Check if the dump is already suspended
        if (isSuspended) {
            return isSuspended;
        } else {
            isSuspended = true;
            return false;
        }
    }

    /** Resume the dump, only if it was not already suspended.
     * @param wasSuspended flag to tell if the dump was already suspended (true; false otherwise)
     */
    public static void resume(final Boolean wasSuspended) {
        if (!wasSuspended) {
            isSuspended = false;
        }
    }

    /** In case dump is suspended and an exception is thrown,
     * allows the dump to end nicely.
     */
    public static void endNicely() {
        isSuspended = false;
        if (isActive()) deactivate();

    }

    /** Check if dump is active for this thread.
     * @return true if dump is active for this thread
     */
    public static boolean isActive() {
        return DUMP.get() != null && !isSuspended;
    }

    /** Dump DEM cell data.
     * @param tile tile to which the cell belongs
     * @param latitudeIndex latitude index of the cell
     * @param longitudeIndex longitude index of the cell
     * @param elevation elevation of the cell
     */
    public static void dumpTileCell(final Tile tile,
                                    final int latitudeIndex, final int longitudeIndex,
                                    final double elevation) {
        if (isActive()) {
            DUMP.get().dumpTileCell(tile, latitudeIndex, longitudeIndex, elevation);
        }
    }

    /** Dump algorithm data.
     * @param algorithmId algorithm ID
     */
    public static void dumpAlgorithm(final AlgorithmId algorithmId) {
        if (isActive()) {
            DUMP.get().dumpAlgorithm(algorithmId);
        }
    }

    /** Dump algorithm data.
     * @param algorithmId algorithm ID
     * @param specific algorithm specific extra data
     */
    public static void dumpAlgorithm(final AlgorithmId algorithmId, final double specific) {
        if (isActive()) {
            DUMP.get().dumpAlgorithm(algorithmId, specific);
        }
    }

    /** Dump ellipsoid data.
     * @param ellipsoid ellipsoid to dump
     */
    public static void dumpEllipsoid(final ExtendedEllipsoid ellipsoid) {
        if (isActive()) {
            DUMP.get().dumpEllipsoid(ellipsoid);
        }
    }

    /** Dump a direct location computation.
     * @param date date of the location
     * @param sensorPosition sensor position in spacecraft frame
     * @param los normalized line-of-sight in spacecraft frame
     * @param lightTimeCorrection flag for light time correction
     * @param aberrationOfLightCorrection flag for aberration of light correction
     * @param refractionCorrection flag for refraction correction
     */
    public static void dumpDirectLocation(final AbsoluteDate date, final Vector3D sensorPosition, final Vector3D los,
                                          final boolean lightTimeCorrection, final boolean aberrationOfLightCorrection,
                                          final boolean refractionCorrection) {
        if (isActive()) {
            DUMP.get().dumpDirectLocation(date, sensorPosition, los, lightTimeCorrection, aberrationOfLightCorrection,
                                          refractionCorrection);
        }
    }

    /** Dump a direct location result.
     * @param gp resulting geodetic point
     */
    public static void dumpDirectLocationResult(final GeodeticPoint gp) {
        if (isActive()) {
            DUMP.get().dumpDirectLocationResult(gp);
        }
    }

    /** Dump an inverse location computation.
     * @param sensor sensor
     * @param point point to localize
     * @param ellipsoid the used ellipsoid
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @param lightTimeCorrection flag for light time correction
     * @param aberrationOfLightCorrection flag for aberration of light correction
     * @param refractionCorrection flag for refraction correction
     */
    public static void dumpInverseLocation(final LineSensor sensor, final GeodeticPoint point,
                                           final ExtendedEllipsoid ellipsoid,
                                           final int minLine, final int maxLine,
                                           final boolean lightTimeCorrection, final boolean aberrationOfLightCorrection,
                                           final boolean refractionCorrection) {
        if (isActive()) {
            DUMP.get().dumpInverseLocation(sensor, point, minLine, maxLine,
                                           lightTimeCorrection, aberrationOfLightCorrection, refractionCorrection);
            DUMP.get().dumpEllipsoid(ellipsoid);
        }
    }

    /** Dump an inverse location result.
     * @param pixel resulting sensor pixel
     */
    public static void dumpInverseLocationResult(final SensorPixel pixel) {
        if (isActive()) {
            DUMP.get().dumpInverseLocationResult(pixel);
        }
    }

    /** Dump an observation transform transform.
     * @param scToBody provider for observation
     * @param index index of the transform
     * @param bodyToInertial transform from body frame to inertial frame
     * @param scToInertial transfrom from spacecraft frame to inertial frame
     */
    public static void dumpTransform(final SpacecraftToObservedBody scToBody, final int index,
                                     final Transform bodyToInertial, final Transform scToInertial) {
        if (isActive()) {
            DUMP.get().dumpTransform(scToBody, index, bodyToInertial, scToInertial);
        }
    }

    /** Dump a sensor mean plane.
     * @param meanPlane mean plane associated with sensor
     */
    public static void dumpSensorMeanPlane(final SensorMeanPlaneCrossing meanPlane) {
        if (isActive()) {
            DUMP.get().dumpSensorMeanPlane(meanPlane);
        }
    }

    /** Dump a sensor LOS.
     * @param sensor sensor
     * @param date date
     * @param i pixel index
     * @param los pixel normalized line-of-sight
     */
    public static void dumpSensorLOS(final LineSensor sensor, final AbsoluteDate date, final int i, final Vector3D los) {
        if (isActive()) {
            DUMP.get().dumpSensorLOS(sensor, date, i, los);
        }
    }

    /** Dump a sensor datation.
     * @param sensor sensor
     * @param lineNumber line number
     * @param date date
     */
    public static void dumpSensorDatation(final LineSensor sensor, final double lineNumber, final AbsoluteDate date) {
        if (isActive()) {
            DUMP.get().dumpSensorDatation(sensor, lineNumber, date);
        }
    }

    /** Dump a sensor rate.
     * @param sensor sensor
     * @param lineNumber line number
     * @param rate lines rate
     */
    public static void dumpSensorRate(final LineSensor sensor, final double lineNumber, final double rate) {
        if (isActive()) {
            DUMP.get().dumpSensorRate(sensor, lineNumber, rate);
        }
    }

}
