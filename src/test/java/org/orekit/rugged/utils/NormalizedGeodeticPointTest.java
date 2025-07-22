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

import org.hipparchus.util.FastMath;
import org.junit.Assert;
import org.junit.Test;

public class NormalizedGeodeticPointTest {

    /**
     * check {@link NormalizedGeodeticPoint#equals(Object)}.
     */
    @Test
    public void testEquals() {
        // setup
        NormalizedGeodeticPoint point = new NormalizedGeodeticPoint(1, 2, 3, 4);

        // actions + verify
        Assert.assertEquals(point, new NormalizedGeodeticPoint(1, 2, 3, 4));
        Assert.assertFalse(point.equals(new NormalizedGeodeticPoint(0, 2, 3, 4)));
        Assert.assertFalse(point.equals(new NormalizedGeodeticPoint(1, 0, 3, 4)));
        Assert.assertFalse(point.equals(new NormalizedGeodeticPoint(1, 2, 0, 4)));
        Assert.assertFalse(point.equals(new NormalizedGeodeticPoint(1, 2, 3, 10)));
        Assert.assertFalse(point.equals(new Object()));
    }
    
    /**
     * check {@link NormalizedGeodeticPoint#hashCode()}.
     */
    @Test
    public void testHashCode() {
        // setup
        NormalizedGeodeticPoint point = new NormalizedGeodeticPoint(1, 2, 3, 4);

        // actions + verify
        Assert.assertEquals(point.hashCode(), new NormalizedGeodeticPoint(1, 2, 3, 4).hashCode());
        Assert.assertNotEquals(point.hashCode(), new NormalizedGeodeticPoint(1, FastMath.nextUp(2), 3, 4).hashCode());
    }
}
