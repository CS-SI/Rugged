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
package org.orekit.rugged.errors;


import java.util.Locale;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class RuggedExceptionTest {

    @Test
    public void testTranslation() {
        RuggedException re = new RuggedException(RuggedMessages.DUPLICATED_PARAMETER_NAME, "dummy");
        Assertions.assertFalse(re.getMessage(Locale.FRENCH).contains("parameter"));
        Assertions.assertTrue(re.getMessage(Locale.FRENCH).contains("paramètre"));
        Assertions.assertTrue(re.getMessage(Locale.FRENCH).contains("dummy"));
        Assertions.assertTrue(re.getMessage(Locale.US).contains("parameter"));
        Assertions.assertFalse(re.getMessage(Locale.US).contains("paramètre"));
        Assertions.assertTrue(re.getMessage(Locale.US).contains("dummy"));
        Assertions.assertEquals(re.getMessage(), re.getMessage(Locale.US));
    }

    @Test
    public void testParameters() {
        RuggedException re = new RuggedException(RuggedMessages.DUPLICATED_PARAMETER_NAME, "dummy");
        Assertions.assertEquals(RuggedMessages.DUPLICATED_PARAMETER_NAME, re.getSpecifier());
        Assertions.assertEquals("dummy", re.getParts()[0]);
    }

    @Test
    public void testNullSpecifier() {
        RuggedException re = new RuggedException(null, (Object[]) null);
        Assertions.assertEquals("", re.getMessage());
    }

    @Test
    public void testNullParts() {
        RuggedException re1 = new RuggedException(RuggedMessages.NO_PARAMETERS_SELECTED, (Object[]) null);
        Assertions.assertEquals(RuggedMessages.NO_PARAMETERS_SELECTED, re1.getSpecifier());
        Assertions.assertEquals(0, re1.getParts().length);
        RuggedException re2 = new RuggedException(new RuntimeException(),
                                                  RuggedMessages.NO_PARAMETERS_SELECTED, (Object[]) null);
        Assertions.assertEquals(RuggedMessages.NO_PARAMETERS_SELECTED, re2.getSpecifier());
        Assertions.assertEquals(0, re2.getParts().length);
    }

    @Test
    public void testInternalError() {
        RuggedException re = new RuggedException(RuggedMessages.DUPLICATED_PARAMETER_NAME, "dummy");
        RuntimeException rte = new RuggedInternalError(re);
        Assertions.assertFalse(re.getLocalizedMessage().contains("https://gitlab.orekit.org/orekit/rugged/issues"));
        Assertions.assertTrue(rte.getLocalizedMessage().contains("https://gitlab.orekit.org/orekit/rugged/issues"));
        Assertions.assertTrue(rte.getMessage().contains("https://gitlab.orekit.org/orekit/rugged/issues"));
    }

    @Deprecated
    @Test
    public void testCoverage() {
        RuggedExceptionWrapper rew = new RuggedExceptionWrapper(new RuggedException(RuggedMessages.DUPLICATED_PARAMETER_NAME, "dummy"));
        RuggedException re = rew.getException();
        Assertions.assertEquals(RuggedMessages.DUPLICATED_PARAMETER_NAME, re.getSpecifier());
        Assertions.assertEquals("dummy", re.getParts()[0]);
    }
}
