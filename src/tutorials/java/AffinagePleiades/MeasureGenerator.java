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


import org.orekit.rugged.api.SensorToGroundMapping;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.time.AbsoluteDate;
import org.orekit.bodies.GeodeticPoint;

/** class for measure generation
 * @author Jonathan Guinet
 */
public class MeasureGenerator {


    /** mapping */
    private SensorToGroundMapping mapping;

    private Rugged rugged;
    
    private LineSensor sensor;

    private PleiadesViewingModel viewingModel;
    
    
    
    /** Simple constructor.
     * <p>
     *
     * </p>
     */
    public MeasureGenerator(PleiadesViewingModel viewingModel, Rugged rugged) throws RuggedException
    {
	
    // generate reference mapping
    String sensorName = viewingModel.getSensorName();	
    mapping = new SensorToGroundMapping(sensorName);
    this.rugged = rugged;
    this.viewingModel = viewingModel;
    sensor = rugged.getLineSensor(mapping.getSensorName());
    
    }
    
    public SensorToGroundMapping getMapping() {
    	return mapping;
    }
    
    
    public void CreateMeasure(final int lineSampling,final int pixelSampling)  throws RuggedException
    {
    
    for (double line = 0; line < viewingModel.dimension; line += lineSampling) {
    	
        AbsoluteDate date = sensor.getDate(line);
        for (int pixel = 0; pixel < sensor.getNbPixels(); pixel += pixelSampling) {
            GeodeticPoint gp2 = rugged.directLocation(date, sensor.getPosition(),
                                                      sensor.getLOS(date, pixel));
            mapping.addMapping(new SensorPixel(line, pixel), gp2);
        }
    }
    }
	

    
}

