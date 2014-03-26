/* Copyright 2002-2014 CS Systèmes d'Information
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
package org.orekit.rugged.core.duvenhage;


import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.junit.Assert;
import org.junit.Test;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.core.AbstractAlgorithmTest;
import org.orekit.rugged.core.raster.IntersectionAlgorithm;

public class DuvenhageAlgorithmTest extends AbstractAlgorithmTest {

    protected IntersectionAlgorithm createAlgorithm() {
        return new DuvenhageAlgorithm();
    }

    @Test
    public void testNumericalIssueAtTileExit() throws RuggedException, OrekitException {
        setUpMayonVolcanoContext();
        final IntersectionAlgorithm algorithm = createAlgorithm();
        algorithm.setUpTilesManagement(updater, 8);
        GeodeticPoint intersection = algorithm.intersection(earth,
                                                            new Vector3D(-3787079.6453602533,
                                                                          5856784.405679551,
                                                                          1655869.0582939098),
                                                            new Vector3D( 0.5127552821932051,
                                                                         -0.8254313129088879,
                                                                         -0.2361041470463311));
        Assert.assertEquals(16.0, intersection.getAltitude(), 1.0e-10);
    }

}
