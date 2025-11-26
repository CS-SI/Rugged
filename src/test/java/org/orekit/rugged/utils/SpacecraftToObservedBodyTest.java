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
package org.orekit.rugged.utils;

import java.io.File;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.orekit.bodies.BodyShape;
import org.orekit.data.DataContext;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.Orbit;
import org.orekit.rugged.TestUtils;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.LinearLineDatation;
import org.orekit.rugged.los.LOSBuilder;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.Constants;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;


public class SpacecraftToObservedBodyTest {

	public static Collection<Object[]> data() {
		return Arrays.asList(new Object[][] {
			// Run multiple times the tests, constructing this class with the following parameters
			{ +10, +1., -1., +1}, { -1., -10., -1., +1}, { -1., +1., +15., +1}, { -1., +1., -1., -15.}
		});
	}
	   
	// configuration of the run : shift of date for PV and Q (min and max values)
	private double shiftPVmin;
	private double shiftPVmax;
	private double shiftQmin;
	private double shiftQmax;


    @MethodSource("data")
    @ParameterizedTest
    public void testIssue256(double shiftPVmin, double shiftPVmax, double shiftQmin, double shiftQmax) {

        initSpacecraftToObservedBodyTest(shiftPVmin, shiftPVmax, shiftQmin, shiftQmax);
        
        AbsoluteDate minSensorDate = sensor.getDate(0);
        AbsoluteDate maxSensorDate = sensor.getDate(2000);
        
        AbsoluteDate minPVdate = minSensorDate.shiftedBy(this.shiftPVmin);
        AbsoluteDate maxPVdate = maxSensorDate.shiftedBy(this.shiftPVmax);
        List<TimeStampedPVCoordinates> pvList = TestUtils.orbitToPV(orbit, earth, minPVdate, maxPVdate, 0.25);
        
        AbsoluteDate minQdate = minSensorDate.shiftedBy(this.shiftQmin);
        AbsoluteDate maxQdate = maxSensorDate.shiftedBy(this.shiftQmax);
        List<TimeStampedAngularCoordinates> qList = TestUtils.orbitToQ(orbit, earth, minQdate, maxQdate, 0.25);
        
        try {
        
    	new SpacecraftToObservedBody(FramesFactory.getEME2000(), earth.getBodyFrame(),
                minSensorDate, maxSensorDate, 0.01,
                5.0,
                pvList,
                8, CartesianDerivativesFilter.USE_PV,
                qList,
                2, AngularDerivativesFilter.USE_R);
        Assertions.fail("an exception should have been thrown");
    	
        } catch (RuggedException re){
            Assertions.assertEquals(RuggedMessages.OUT_OF_TIME_RANGE, re.getSpecifier());
        }        	
    }


    public void initSpacecraftToObservedBodyTest(double shiftPVmin, double shiftPVmax, double shiftQmin, double shiftQmax) {
		this.shiftPVmin = shiftPVmin;
		this.shiftPVmax = shiftPVmax;
		this.shiftQmin = shiftQmin;
		this.shiftQmax = shiftQmax;
	}
	

    @BeforeEach
    public void setUp() {
        try {

            String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
            DataContext.getDefault().getDataProvidersManager().addProvider(new DirectoryCrawler(new File(path)));

            earth = TestUtils.createEarth();
            orbit = TestUtils.createOrbit(Constants.EIGEN5C_EARTH_MU);
            
            // build lists of pixels regularly spread on a perfect plane
            final Vector3D position  = new Vector3D(1.5, Vector3D.PLUS_I);
            final Vector3D normal    = Vector3D.PLUS_I;
            final Vector3D fovCenter = Vector3D.PLUS_K;
            final Vector3D cross     = Vector3D.crossProduct(normal, fovCenter);
            
            final List<Vector3D> los = new ArrayList<Vector3D>();
            for (int i = -1000; i <= 1000; ++i) {
                final double alpha = i * 0.17 / 1000;
                los.add(new Vector3D(FastMath.cos(alpha), fovCenter, FastMath.sin(alpha), cross));
            }
            sensor = new LineSensor("perfect line", new LinearLineDatation(AbsoluteDate.J2000_EPOCH, 0.0, 1.0 / 1.5e-3),
                                    position, new LOSBuilder(los).build());
            

        } catch (OrekitException oe) {
            Assertions.fail(oe.getLocalizedMessage());
        } catch (URISyntaxException use) {
            Assertions.fail(use.getLocalizedMessage());
        }
    }

    @AfterEach
    public void tearDown() {
    }

	private BodyShape  earth = null;
	private Orbit      orbit = null;
	private LineSensor sensor = null;
	
}
