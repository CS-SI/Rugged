/* Copyright 2013-2014 CS Systèmes d'Information
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
package org.orekit.rugged.api;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.random.RandomGenerator;
import org.apache.commons.math3.random.Well19937a;
import org.apache.commons.math3.util.FastMath;
import org.junit.Assert;
import org.junit.Test;
import org.orekit.rugged.api.LinearLineDatation;
import org.orekit.rugged.api.LineSensor;
import org.orekit.time.AbsoluteDate;

public class LineSensorTest {

    @Test
    public void testPerfectLine() {

        final Vector3D       position  = new Vector3D(1.5, Vector3D.PLUS_I);
        final Vector3D       normal    = Vector3D.PLUS_I;
        final Vector3D       fovCenter = Vector3D.PLUS_K;
        final Vector3D       cross     = Vector3D.crossProduct(normal, fovCenter);

        // build lists of pixels regularly spread on a perfect plane
        final List<Vector3D> los       = new ArrayList<Vector3D>();
        for (int i = -1000; i <= 1000; ++i) {
            final double alpha = i * 0.17 / 1000;
            los.add(new Vector3D(FastMath.cos(alpha), fovCenter, FastMath.sin(alpha), cross));
        }

        final LineSensor sensor = new LineSensor("perfect line",
                                         new LinearLineDatation(AbsoluteDate.J2000_EPOCH, 0.0, 1.0 / 1.5e-3),
                                         position, los);

        Assert.assertEquals("perfect line", sensor.getName());
        Assert.assertEquals(AbsoluteDate.J2000_EPOCH, sensor.getDate(0.0));
        Assert.assertEquals(0.0, Vector3D.distance(position, sensor.getPosition()), 1.0e-15);
        Assert.assertEquals(0.0, Vector3D.angle(normal, sensor.getMeanPlaneNormal()), 1.0e-15);

    }

    @Test
    public void testNoisyLine() {

        final RandomGenerator random    = new Well19937a(0xf3ddb33785e12bdal);
        final Vector3D        position  = new Vector3D(1.5, Vector3D.PLUS_I);
        final Vector3D        normal    = Vector3D.PLUS_I;
        final Vector3D        fovCenter = Vector3D.PLUS_K;
        final Vector3D        cross     = Vector3D.crossProduct(normal, fovCenter);

        // build lists of pixels regularly spread on a perfect plane
        final List<Vector3D> los       = new ArrayList<Vector3D>();
        for (int i = -1000; i <= 1000; ++i) {
            final double alpha = i * 0.17 / 10 + 1.0e-5 * random.nextDouble();
            final double delta = 1.0e-5 * random.nextDouble();
            final double cA = FastMath.cos(alpha);
            final double sA = FastMath.sin(alpha);
            final double cD = FastMath.cos(delta);
            final double sD = FastMath.sin(delta);
            los.add(new Vector3D(cA * cD, fovCenter, sA * cD, cross, sD, normal));
        }

        final LineSensor sensor = new LineSensor("noisy line",
                                                 new LinearLineDatation(AbsoluteDate.J2000_EPOCH, 0.0, 1.0 / 1.5e-3),
                                                 position, los);

        Assert.assertEquals("noisy line", sensor.getName());
        Assert.assertEquals(AbsoluteDate.J2000_EPOCH, sensor.getDate(0.0));
        Assert.assertEquals(0.0, Vector3D.distance(position, sensor.getPosition()), 1.0e-5);
        Assert.assertEquals(0.0, Vector3D.angle(normal, sensor.getMeanPlaneNormal()), 3.0e-7);

    }

}
