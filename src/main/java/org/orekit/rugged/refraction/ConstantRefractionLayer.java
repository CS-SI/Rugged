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
package org.orekit.rugged.refraction;

/**
 * Class that represents a constant refraction layer to be used with {@link MultiLayerModel}.
 * @author Sergio Esteves
 */
public class ConstantRefractionLayer implements Comparable<ConstantRefractionLayer> {

    /** lowest altitude of this layer. */
    private final Double lowestAltitude;

    /** refractive index of this layer. */
    private final double refractiveIndex;

    /** Simple constructor.
     * @param lowestAltitude lowest altitude of the layer
     * @param refractiveIndex refractive index of the layer
     */
    public ConstantRefractionLayer(final double lowestAltitude, final double refractiveIndex) {
        this.lowestAltitude = lowestAltitude;
        this.refractiveIndex = refractiveIndex;
    }

    public double getLowestAltitude() {
        return lowestAltitude;
    }

    public double getRefractiveIndex() {
        return refractiveIndex;
    }

    @Override
    public int compareTo(final ConstantRefractionLayer o) {
        return lowestAltitude.compareTo(o.lowestAltitude);
    }

    @Override
    public boolean equals(final Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        final ConstantRefractionLayer that = (ConstantRefractionLayer) o;

        if (Double.compare(that.refractiveIndex, refractiveIndex) != 0) return false;
        return lowestAltitude != null ? lowestAltitude.equals(that.lowestAltitude) : that.lowestAltitude == null;

    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = lowestAltitude != null ? lowestAltitude.hashCode() : 0;
        temp = Double.doubleToLongBits(refractiveIndex);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }
}
