/* Copyright 2013-2017 CS Systèmes d'Information
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
package fr.cs.examples.refiningPleiades.generators;


/** Class for adding a noise to measurements.
 * @author Lucie Labat-Allee
 * @author Guylaine Prat
 * @since 2.0
 */
public class Noise {

    /** Type of distribution. */
    private static final int GAUSSIAN = 0;

    /** Dimension. */
    private int dimension;

    /** Mean. */
    private double[] mean;

    /** Standard deviation. */
    private double[] standardDeviation;

    /** Distribution. */
    private int distribution = GAUSSIAN;

    /** Build a new instance.
     * @param distribution noise type
     * @param dimension noise dimension
     */
    public Noise(final int distribution, final int dimension) {

        this.mean = new double[dimension];
        this.standardDeviation = new double[dimension];
        this.dimension = dimension;
        this.distribution = distribution;
    }

    /** Get the mean.
     * @return the mean
     */
    public double[] getMean() {
        return mean.clone();
    }

    /** Set the mean.
     * @param meanValue the mean to set
     */
    public void setMean(final double[] meanValue) {
        this.mean = meanValue.clone();
    }

    /** Get the standard deviation.
     * @return the standard deviation
     */
    public double[] getStandardDeviation() {
        return standardDeviation.clone();
    }

    /** Set the standard deviation.
     * @param standardDeviationValue the standard deviation to set
     */
    public void setStandardDeviation(final double[] standardDeviationValue) {
        this.standardDeviation = standardDeviationValue.clone();
    }

    /** Get the distribution.
     * @return the distribution
     */
    public int getDistribution() {
        return distribution;
    }

    /** Get the dimension.
     * @return the dimension
     */
    public int getDimension() {
        return dimension;
    }
}
