/* Copyright 2013-2015 CS Systèmes d'Information
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
package org.orekit.rugged.errors;


import java.io.File;
import java.io.IOException;
import java.net.URISyntaxException;

import org.junit.Assert;
import org.junit.Test;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.rugged.api.Rugged;

public class DumpReplayerTest {

    @Test
    public void testDirectLoc01() throws URISyntaxException, IOException, OrekitException, RuggedException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        String dumpPath = getClass().getClassLoader().getResource("replay/replay-direct-loc-01.txt").toURI().getPath();
        DumpReplayer replayer = new DumpReplayer();
        replayer.parse(new File(dumpPath));
        Rugged rugged = replayer.createRugged();
        Object[] results = replayer.execute(rugged);
        Assert.assertEquals(3, results.length);
        Assert.assertTrue(results[0] instanceof GeodeticPoint);
        Assert.assertTrue(results[1] instanceof GeodeticPoint);
        Assert.assertTrue(results[2] instanceof GeodeticPoint);
    }

}
