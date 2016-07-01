package org.orekit.rugged.atmosphericrefraction;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

/**
 * Multi layer model for atmospheric refraction.
 * @author Sergio Esteves
 */
public class MultiLayerModel implements AtmosphericRefraction {

    private static final double KARMA_LINE = 1000000;
    private static final double LAYER_SIZE = KARMA_LINE * 0.25;

    private int numberOfLayers = 1;

    public MultiLayerModel() {
    }

    public MultiLayerModel(int numberOfLayers) {
        this.numberOfLayers = numberOfLayers;
    }

    @Override
    public long getDeviation(Vector3D pos, Vector3D los, double altitude) {

        double numberOfCrossedLayers = (KARMA_LINE - altitude) / LAYER_SIZE;



        return 0;
    }
}
