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
package org.orekit.rugged.raster;

import org.orekit.rugged.raster.SimpleTile;
import org.orekit.rugged.raster.SimpleTileFactory;
import org.orekit.rugged.raster.TileFactory;

public class CountingFactory implements TileFactory<SimpleTile> {

    private int count;
    private TileFactory<SimpleTile> rawFactory;

    public CountingFactory() {
        count = 0;
        rawFactory = new SimpleTileFactory();
    }

    public SimpleTile createTile() {
        ++count;
        return rawFactory.createTile();
    }

    public int getCount() {
        return count;
    }

}
