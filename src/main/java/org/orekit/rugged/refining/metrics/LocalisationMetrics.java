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
package org.orekit.rugged.refining.metrics;

import org.orekit.rugged.refining.measures.SensorToGroundMapping;
import org.orekit.rugged.refining.models.PleiadesViewingModel;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.time.AbsoluteDate;

import java.util.Map;
import java.util.Set;
import java.util.Iterator;

import org.orekit.bodies.GeodeticPoint;

/**
 * class for measure generation
 * 
 * @author Jonathan Guinet
 */
public class LocalisationMetrics {

	/** mapping */
	private Set<Map.Entry<SensorPixel, GeodeticPoint>> groundTruthMappings;

	private Set<Map.Entry<SensorPixel, GeodeticPoint>> estimationMappings;

	private Rugged rugged;

	private LineSensor sensor;

	private PleiadesViewingModel viewingModel;

	private int measureCount;

	private boolean computeInDeg;

	/* max residual distance */
	private double resMax;

	/* mean residual distance */
	private double resMean;

	/**
	 * Simple constructor.
	 * <p>
	 *
	 * </p>
	 */
	public LocalisationMetrics(SensorToGroundMapping groundTruthMapping, Rugged rugged, boolean computeInDeg)
			throws RuggedException {

		groundTruthMappings = groundTruthMapping.getMapping();
		this.rugged = rugged;
		this.sensor = rugged.getLineSensor(groundTruthMapping.getSensorName());
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

	public void computeMetrics() throws RuggedException {

		// final RealVector longDiffVector;
		// final RealVector latDiffVector;
		// final RealVector altDiffVector;

		double count = 0;
		resMax = 0;
		int k = groundTruthMappings.size();
		Iterator<Map.Entry<SensorPixel, GeodeticPoint>> gtIt = groundTruthMappings.iterator();

		while (gtIt.hasNext()) {
			Map.Entry<SensorPixel, GeodeticPoint> gtMapping = gtIt.next();

			final SensorPixel gtSP = gtMapping.getKey();
			final GeodeticPoint gtGP = gtMapping.getValue();

			AbsoluteDate date = sensor.getDate(gtSP.getLineNumber());

			GeodeticPoint esGP = rugged.directLocation(date, sensor.getPosition(),
					sensor.getLOS(date, (int) gtSP.getPixelNumber()));

			double distance = 0;

			if (this.computeInDeg == true) {
				double lonDiff = esGP.getLongitude() - gtGP.getLongitude();
				double latDiff = esGP.getLatitude() - gtGP.getLatitude();
				double altDiff = esGP.getAltitude() - gtGP.getAltitude();
				distance = Math.sqrt(lonDiff * lonDiff + latDiff * latDiff);
			} else {

				distance = DistanceTools.computeDistanceRad(esGP.getLongitude(), esGP.getLatitude(),
						gtGP.getLongitude(), gtGP.getLatitude());
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
		// System.out.format("number of points %d %n", k);
	}

}
