/* Copyright 2013-2019 CS Systèmes d'Information
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


import java.util.Locale;

import org.junit.Assert;
import org.junit.Test;

public class RuggedExceptionTest {

    @Test
    public void testTranslation() {
        RuggedException re = new RuggedException(RuggedMessages.DUPLICATED_PARAMETER_NAME, "dummy");
        Assert.assertFalse(re.getMessage(Locale.FRENCH).contains("parameter"));
        Assert.assertTrue(re.getMessage(Locale.FRENCH).contains("paramètre"));
        Assert.assertTrue(re.getMessage(Locale.FRENCH).contains("dummy"));
        Assert.assertTrue(re.getMessage(Locale.US).contains("parameter"));
        Assert.assertFalse(re.getMessage(Locale.US).contains("paramètre"));
        Assert.assertTrue(re.getMessage(Locale.US).contains("dummy"));
        Assert.assertEquals(re.getMessage(), re.getMessage(Locale.US));
    }

    @Test
    public void testParameters() {
        RuggedException re = new RuggedException(RuggedMessages.DUPLICATED_PARAMETER_NAME, "dummy");
        Assert.assertEquals(RuggedMessages.DUPLICATED_PARAMETER_NAME, re.getSpecifier());
        Assert.assertEquals("dummy", re.getParts()[0]);
    }

    @Test
    public void testNullSpecifier() {
        RuggedException re = new RuggedException(null, (Object[]) null);
        Assert.assertEquals("", re.getMessage());
    }

    @Test
    public void testNullParts() {
        RuggedException re1 = new RuggedException(RuggedMessages.NO_PARAMETERS_SELECTED, (Object[]) null);
        Assert.assertEquals(RuggedMessages.NO_PARAMETERS_SELECTED, re1.getSpecifier());
        Assert.assertEquals(0, re1.getParts().length);
        RuggedException re2 = new RuggedException(new RuntimeException(),
                                                  RuggedMessages.NO_PARAMETERS_SELECTED, (Object[]) null);
        Assert.assertEquals(RuggedMessages.NO_PARAMETERS_SELECTED, re2.getSpecifier());
        Assert.assertEquals(0, re2.getParts().length);
    }

    @Test
    public void testInternalError() {
        RuggedException re = new RuggedException(RuggedMessages.DUPLICATED_PARAMETER_NAME, "dummy");
        RuntimeException rte = RuggedException.createInternalError(re);
        Assert.assertFalse(re.getLocalizedMessage().contains("https://gitlab.orekit.org/orekit/rugged/issues"));
        Assert.assertTrue(rte.getLocalizedMessage().contains("https://gitlab.orekit.org/orekit/rugged/issues"));
        Assert.assertTrue(rte.getMessage().contains("https://gitlab.orekit.org/orekit/rugged/issues"));
    }

}
