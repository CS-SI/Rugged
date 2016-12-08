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
package AffinagePleiades;

import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.RealVector;

import org.orekit.rugged.api.SensorToSensorMapping;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorMeanPlaneCrossing;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.utils.SpacecraftToObservedBody;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.time.AbsoluteDate;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import java.util.Collections;
import java.util.Collection;
import java.util.Map;
import java.util.Set;
import java.util.Iterator;
import java.util.Locale;

import org.hipparchus.util.FastMath;
import org.orekit.bodies.GeodeticPoint;

/**
 * class for measure generation
 * 
 * @author Jonathan Guinet
 */
public class SensorToSensorLocalisationMetrics {

	/** mapping */
    private SensorToSensorMapping  mapping;
    
	private Set<Map.Entry<SensorPixel, SensorPixel>> groundTruthMappings;
	
	
	
	private Rugged ruggedA;
    private Rugged ruggedB;
    
	private LineSensor sensorA;
    private LineSensor sensorB;
    

	private boolean computeInDeg;

	/* max residual distance */
	private double resMax;

	/* mean residual distance */
	private double resMean;

    /* los distance max */
    private double losMax;

    /* los distance mean*/
    private double losMean;
    
    /* earth distance max */
    private double earthDistanceMax;

    /* earth distance mean*/
    private double earthDistanceMean; 
	
	
	/**
	 * Simple constructor.
	 * <p>
	 *
	 * </p>
	 */
	public SensorToSensorLocalisationMetrics(SensorToSensorMapping mapping, Rugged ruggedA, Rugged ruggedB , boolean computeInDeg)
			throws RuggedException {

	    this.mapping = mapping;
		groundTruthMappings = mapping.getMappings();
		
		this.ruggedA = ruggedA;
        this.ruggedB = ruggedB;
        
		this.sensorA = ruggedA.getLineSensor(mapping.getSensorNameA());
	    this.sensorB = ruggedB.getLineSensor(mapping.getSensorNameB());
	    
		this.computeInDeg = computeInDeg;
		this.computeMetrics();
	}

	/**
	 * Get the maximum residual;
	 * 
	 * @return max residual
	 */
	public double getMaxResidual() {
		return resMax;
	}

	/**
	 * Get the mean residual;
	 * 
	 * @return mean residual
	 */
	public double getMeanResidual() {
		return resMean;
	}

    /**
     * Get the los maximum residual;
     * 
     * @return max residual
     */
    public double getLosMaxDistance() {
        return losMax;
    }

    /**
     * Get the los mean residual;
     * 
     * @return mean residual
     */
    public double getLosMeanDistance() {
        return losMean;
    }

    /**
     * Get the earth distance maximum residual;
     * 
     * @return max residual
     */
    public double getEarthMaxDistance() {
        return earthDistanceMax;
    }

    /**
     * Get the earth distance mean residual;
     * 
     * @return mean residual
     */
    public double getEarthMeanDistance() {
        return earthDistanceMean;
    }    
    
    
	
	
	public void computeMetrics() throws RuggedException {

		// final RealVector longDiffVector;
		// final RealVector latDiffVector;
		// final RealVector altDiffVector;

		double count = 0;
		double losCount = 0;
        double earthDistanceCount = 0;
        
		resMax = 0;
		losMax = 0;
		earthDistanceMax = 0;
		
		int k = groundTruthMappings.size();
		Iterator<Map.Entry<SensorPixel, SensorPixel>> gtIt = groundTruthMappings.iterator();
		while (gtIt.hasNext()) {
			Map.Entry<SensorPixel, SensorPixel> gtMapping = gtIt.next();
			
			
			final SensorPixel spA = gtMapping.getKey();
			final SensorPixel spB = gtMapping.getValue();
			
			Double [] gtDistance = mapping.getDistance(spA)
			                ; 
			AbsoluteDate dateA = sensorA.getDate(spA.getLineNumber());
            AbsoluteDate dateB = sensorB.getDate(spB.getLineNumber());

            final double pixelA = spA.getPixelNumber();
            final double pixelB = spB.getPixelNumber();
            /*System.out.format(" %n");
            System.out.format("pix A %f %f %n", spA.getLineNumber(),
                              spA.getPixelNumber());
            System.out.format("pix B %f %f %n", spB.getLineNumber(),
                              spB.getPixelNumber());*/
            
            Vector3D losA = sensorA.getLOS(dateA, pixelA);
            Vector3D losB = sensorB.getLOS(dateB, pixelB);
                        
			
			GeodeticPoint gpA = ruggedA.directLocation(dateA, sensorA.getPosition(),losA);
            GeodeticPoint gpB = ruggedB.directLocation(dateB, sensorB.getPosition(),losB);

			double distance = 0;
			
            final SpacecraftToObservedBody scToBodyA = ruggedA.getScToBody();
          
            
			double[] losDistance = ruggedB.distanceBetweenLOS(sensorA,dateA, pixelA, scToBodyA, sensorB ,dateB, pixelB);
			
			//System.out.format("loc distance %1.8e dist %f %n",losDistance[0],losDistance[1]);
			//System.out.format("gt dist  %1.8e d %f %n",gtDistance[0],gtDistance[1]);
			if (losDistance[0] > losMax) {
                losMax = losDistance[0];

            }
            losCount += losDistance[0];
            final double earthDistance =  Math.abs(losDistance[1] - gtDistance[1]);
            //System.out.format( "los Distance %3.6e %3.6e %n",losDistance[1] , gtDistance[1]);
            if (earthDistance > earthDistanceMax) {
                earthDistanceMax = earthDistance;

            }
            earthDistanceCount += earthDistance;
                        
            
            
            
			if (this.computeInDeg == true) {
				double lonDiff = gpB.getLongitude() - gpA.getLongitude();
				double latDiff = gpB.getLatitude() - gpA.getLatitude();
				double altDiff = gpB.getAltitude() - gpA.getAltitude();
				distance = Math.sqrt(lonDiff * lonDiff + latDiff * latDiff);
			} else {

                distance = DistanceTools.computeDistanceRad(gpB.getLongitude(), gpB.getLatitude(),
                                                            gpA.getLongitude(), gpA.getLatitude());
			}
			count += distance;
			if (distance > resMax) {
				resMax = distance;

			}
			// distanceVector.append(distance);

		}

		// resMax = distanceVector.getMaxValue();
		// System.out.format(Locale.US, "max: %3.6e %n
		// ",distanceVector.getMaxValue() )

		resMean = count / k;
		losMean = losCount / k;
		earthDistanceMean = earthDistanceCount / k;
		// System.out.format("number of points %d %n", k);
	}

}
