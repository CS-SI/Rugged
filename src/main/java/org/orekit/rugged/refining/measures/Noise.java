/* Copyright 2013-2016 CS Systèmes d'Information
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
package org.orekit.rugged.refining.measures;


/** Class for adding a noise to measures.
 * @author Lucie Labat-Allee
 * @since 2.0
 */
public class Noise {

    /** Mean */
    private double[] mean;

    /** Standard deviation */
    private double[] standardDeviation;

    /** Type of distribution */
    private static final int GAUSSIAN=0;

    /** Distribution */
    private int distribution=GAUSSIAN;
    
    /** Dimension */
    private static int dimension;
    

    /** Build a new instance.
     * @param 
     * @param 
     */
    public Noise(final int distribution, final int dimension) {
        this.mean = new double[dimension];
        this.standardDeviation = new double[dimension];
        Noise.dimension = dimension;
        this.distribution = distribution;
    }


    
    /**
     * @return the mean
     */
    public double[] getMean() {
        return mean;
    }


    
    /**
     * @param mean the mean to set
     */
    public void setMean(double[] mean) {
        this.mean = mean;
    }


    
    /**
     * @return the standardDeviation
     */
    public double[] getStandardDeviation() {
        return standardDeviation;
    }


    
    /**
     * @param standardDeviation the standardDeviation to set
     */
    public void setStandardDeviation(double[] standardDeviation) {
        this.standardDeviation = standardDeviation;
    }


    
    /**
     * @return the distribution
     */
    public int getDistribution() {
        return distribution;
    }


    
    /**
     * @return the dimension
     */
    public static int getDimension() {
        return dimension;
    }

    
}

 