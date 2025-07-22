/* Copyright 2013-2025 CS GROUP
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
package org.orekit.rugged.refraction;

/**
 * Class that represents a constant refraction layer to be used with {@link MultiLayerModel}.
 * @author Sergio Esteves
 * @author Guylaine Prat
 * @since 2.0
 */
public class ConstantRefractionLayer {

    /** lowest altitude of this layer (m). */
    private final Double lowestAltitude;

    /** refractive index of this layer. */
    private final double refractiveIndex;

    /** Simple constructor.
     * @param lowestAltitude lowest altitude of the layer (m)
     * @param refractiveIndex refractive index of the layer
     */
    public ConstantRefractionLayer(final double lowestAltitude, final double refractiveIndex) {
        this.lowestAltitude = lowestAltitude;
        this.refractiveIndex = refractiveIndex;
    }

    /**
     * @return the lowest altitude of the layer (m)
     */
    public double getLowestAltitude() {
        return lowestAltitude;
    }

    /**
     * @return the refractive index of the layer
     */
    public double getRefractiveIndex() {
        return refractiveIndex;
    }
}
