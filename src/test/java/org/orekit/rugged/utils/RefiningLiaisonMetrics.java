package org.orekit.rugged.utils;

/**
 * Contains results from liaison metrics computation
 */
public class RefiningLiaisonMetrics {
    
    /** Maximum residual distance. */
    private double resMax;
    /** Mean residual distance. */
    private double resMean;
    /** LOS distance max. */
    private double losDistanceMax;
    /** LOS distance mean. */
    private double losDistanceMean;
    /** Earth distance max. */
    private double earthDistanceMax;
    /** Earth distance mean.*/
    private double earthDistanceMean;
    
    public RefiningLiaisonMetrics() {
        
        this.resMax = 0.0;
        this.resMean = 0.0;
        this.losDistanceMax = 0.0;
        this.losDistanceMean = 0.0;
        this.earthDistanceMax = 0.0;
        this.earthDistanceMean = 0.0;
    }

    public double getMaxResidual() {
        return resMax;
    }

    public void setMaxResidual(double resMax) {
        this.resMax = resMax;
    }

    public double getMeanResidual() {
        return resMean;
    }

    public void setMeanResidual(double resMean) {
        this.resMean = resMean;
    }

    public double getLosMaxDistance() {
        return losDistanceMax;
    }

    public void setLosMaxDistance(double losDistanceMax) {
        this.losDistanceMax = losDistanceMax;
    }

    public double getLosMeanDistance() {
        return losDistanceMean;
    }

    public void setLosMeanDistance(double losDistanceMean) {
        this.losDistanceMean = losDistanceMean;
    }

    public double getEarthMaxDistance() {
        return earthDistanceMax;
    }

    public void setEarthMaxDistance(double earthDistanceMax) {
        this.earthDistanceMax = earthDistanceMax;
    }

    public double getEarthMeanDistance() {
        return earthDistanceMean;
    }

    public void setEarthMeanDistance(double earthDistanceMean) {
        this.earthDistanceMean = earthDistanceMean;
    }
}
