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

import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.SingularValueDecomposition;
import org.apache.commons.math3.util.FastMath;
import org.orekit.time.AbsoluteDate;

/** Line sensor model.
 * <p>
 * Instances of this class are guaranteed to be immutable.
 * </p>
 * @author Luc Maisonobe
 */
public class LineSensor {

    /** Name of the sensor. */
    private final String name;

    /** Datation model. */
    private final LineDatation datationModel;

    /** Sensor position. */
    private final Vector3D position;

    /** Pixels lines-of-sight. */
    private final Vector3D[] x;

    /** Pixels transversal direction (i.e. towards left pixel). */
    private final Vector3D[] y;

    /** Pixels widths. */
    private final double[] width;

    /** Mean plane normal. */
    private final Vector3D normal;

    /** Simple constructor.
     * @param name name of the sensor
     * @param position sensor position
     * @param los pixels lines-of-sight
     * @param datationModel datation model
     */
    public LineSensor(final String name, final LineDatation datationModel,
                      final Vector3D position, final List<Vector3D> los) {

        this.name          = name;
        this.datationModel = datationModel;
        this.position      = position;

        // normalize lines-of-sight
        this.x = new Vector3D[los.size()];
        for (int i = 0; i < los.size(); ++i) {
            x[i] = los.get(i).normalize();
        }

        // we consider the viewing directions as a point cloud
        // and want to find the plane that best fits it

        // start by finding the centroid
        double centroidX = 0;
        double centroidY = 0;
        double centroidZ = 0;
        for (int i = 0; i < x.length; ++i) {
            final Vector3D l = x[i];
            centroidX += l.getX();
            centroidY += l.getY();
            centroidZ += l.getZ();
        }
        centroidX /= x.length;
        centroidY /= x.length;
        centroidZ /= x.length;

        // build a centered data matrix
        final RealMatrix matrix = MatrixUtils.createRealMatrix(3, los.size());
        for (int i = 0; i < x.length; ++i) {
            final Vector3D l = x[i];
            matrix.setEntry(0, i, l.getX() - centroidX);
            matrix.setEntry(1, i, l.getY() - centroidY);
            matrix.setEntry(2, i, l.getZ() - centroidZ);
        }

        // compute Singular Value Decomposition
        final SingularValueDecomposition svd = new SingularValueDecomposition(matrix);

        // extract the left singular vector corresponding to least singular value
        // (i.e. last vector since Apache Commons Math returns the values
        //  in non-increasing order)
        final Vector3D singularVector = new Vector3D(svd.getU().getColumn(2)).normalize();

        // check rotation order
        if (Vector3D.dotProduct(singularVector, Vector3D.crossProduct(los.get(0), los.get(los.size() - 1))) >= 0) {
            normal = singularVector;
        } else {
            normal = singularVector.negate();
        }

        // compute transversal direction
        y = new Vector3D[x.length];
        for (int i = 0; i < x.length; ++i) {
            y[i] = Vector3D.crossProduct(normal, x[i]).normalize();
        }

        // compute pixel widths
        width = new double[x.length];
        for (int i = 0; i < x.length; ++i) {
            if (i < 1) {
                width[i] =  getAzimuth(los.get(i + 1), i);
            } else if (i > x.length - 2) {
                width[i] = -getAzimuth(los.get(i - 1), i);
            } else {
                width[i] = 0.5 * (getAzimuth(los.get(i + 1), i) - getAzimuth(los.get(i - 1), i));
            }
        }

    }

    /** Get the name of the sensor.
     * @return name of the sensor
     */
    public String getName() {
        return name;
    }

    /** Get the number of pixels.
     * @return number of pixels
     */
    public int getNbPixels() {
        return x.length;
    }

    /** Get the pixel normalized line-of-sight.
     * @param i pixel index (must be between 0 and {@link #getNbPixels()}
     * @return pixel normalized line-of-sight
     */
    public Vector3D getLos(final int i) {
        return x[i];
    }

    /** Get the date.
     * @param lineNumber line number
     * @return date corresponding to line number
     */
    public AbsoluteDate getDate(final double lineNumber) {
        return datationModel.getDate(lineNumber);
    }

    /** Get the line number.
     * @param date date
     * @return line number corresponding to date
     */
    public double getLine(final AbsoluteDate date) {
        return datationModel.getLine(date);
    }

    /** Get the rate of lines scanning.
     * @param lineNumber line number
     * @return rate of lines scanning (lines / seconds)
     */
    public double getRate(final double lineNumber) {
        return datationModel.getRate(lineNumber);
    }

    /** Get the mean plane normal.
     * <p>
     * The normal is oriented such traversing pixels in increasing indices
     * order corresponds is consistent with trigonometric order (i.e.
     * counterclockwise).
     * </p>
     * @return mean plane normal
     */
    public Vector3D getMeanPlaneNormal() {
        return normal;
    }

    /** Get the sensor position.
     * @return position
     */
    public Vector3D getPosition() {
        return position;
    }

    /** Get the relative azimuth of a direction with respect to a pixel.
     * <p>
     * The relative azimuth is computed along the sensor line. As it is
     * relative to current pixel, it is 0 when the direction is aligned
     * with pixel i, and the sign is consistent with {@link #getMeanPlaneNormal()
     * mean plane normal} orientation.
     * </p>
     * @param direction direction to check
     * @param i pixel index (must be between 0 and {@link #getNbPixels()}
     * @return relative azimuth of direction
     */
    public double getAzimuth(final Vector3D direction, final int i) {
        return FastMath.atan2(Vector3D.dotProduct(direction, y[i]),
                              Vector3D.dotProduct(direction, x[i]));
    }

    /** Get the the angular width a pixel.
     * @param i pixel index (must be between 0 and {@link #getNbPixels()}
     * @return relative azimuth of direction
     */
    public double getWidth(final int i) {
        return width[i];
    }

}
