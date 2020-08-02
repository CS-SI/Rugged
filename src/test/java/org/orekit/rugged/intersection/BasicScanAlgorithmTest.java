/* Copyright 2013-2020 CS GROUP
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
package org.orekit.rugged.intersection;


import static org.junit.Assert.assertEquals;

import org.junit.Test;
import org.orekit.rugged.api.AlgorithmId;
import org.orekit.rugged.raster.TileUpdater;

public class BasicScanAlgorithmTest extends AbstractAlgorithmTest {

    public IntersectionAlgorithm createAlgorithm(final TileUpdater updater, final int maxCachedTiles) {
        return new BasicScanAlgorithm(updater, maxCachedTiles);
    }

    @Test
    public void testAlgorithmId() {
        setUpMayonVolcanoContext();
        final IntersectionAlgorithm algorithm = createAlgorithm(updater, 8);
        assertEquals(AlgorithmId.BASIC_SLOW_EXHAUSTIVE_SCAN_FOR_TESTS_ONLY, algorithm.getAlgorithmId());
    }
}
