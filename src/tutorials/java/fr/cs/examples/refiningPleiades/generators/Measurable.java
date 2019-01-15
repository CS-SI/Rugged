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

/** For measurements generator.
 * @author Lucie Labat-Allee
 * @author Guylaine Prat
 * @since 2.0
 */
public interface Measurable {
	
    /** Get the number of measurements
     * @return the number of measurements
     */
    int  getMeasurementCount();

    /** Create measurements (without noise)
     * @param lineSampling line sampling
     * @param pixelSampling pixel sampling
     */
    void createMeasurement(int lineSampling, int pixelSampling);

    /** Create noisy measurements
     * @param lineSampling line sampling
     * @param pixelSampling pixel sampling
     * @param noise the noise to add to the measurements
     */
    void createNoisyMeasurement(int lineSampling, int pixelSampling, Noise noise);

}
