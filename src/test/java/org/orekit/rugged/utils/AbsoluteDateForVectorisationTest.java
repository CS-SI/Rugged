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


import org.hipparchus.exception.LocalizedCoreFormats;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.orekit.errors.OrekitException;
import org.orekit.time.AbsoluteDate;

public class AbsoluteDateForVectorisationTest {
	@Test
    public void testShiftedBySeveralTimeShiftOneDate() {

		AbsoluteDate date1 = new AbsoluteDate();
        AbsoluteDate[] dates = new AbsoluteDate[] {date1};
        double[] dts = new double[] {10.0, 20.0, 30.0};
	    AbsoluteDateArrayHandling datesForVect = new AbsoluteDateArrayHandling(dates);
	    AbsoluteDate[][] datesShifted = datesForVect.multipleShiftedBy(dts);
	    Assertions.assertEquals(datesShifted[0][0].durationFrom(date1.shiftedBy(10)), 0.0, 1e-5);
	    Assertions.assertEquals(datesShifted[0][1].durationFrom(date1.shiftedBy(20)), 0.0, 1e-5);

    }

	@Test
    public void testShiftedByCorrespondingTimeShift() {

		AbsoluteDate date1 = new AbsoluteDate();
		AbsoluteDate date2 = date1.shiftedBy(10000);
		AbsoluteDate date3 = date1.shiftedBy(20000);
        AbsoluteDate[] dates = new AbsoluteDate[] {date1, date2, date3};
        double[] dts = new double[] {10.0, 20.0, 30.0};
	    AbsoluteDateArrayHandling datesForVect = new AbsoluteDateArrayHandling(dates);
	    AbsoluteDate[] datesShifted = datesForVect.shiftedBy(dts);
	    Assertions.assertEquals(datesShifted[0].durationFrom(date1.shiftedBy(10)), 0.0, 1e-5);
	    Assertions.assertEquals(datesShifted[1].durationFrom(date1.shiftedBy(10020)), 0.0, 1e-5);
	    Assertions.assertEquals(datesShifted[2].durationFrom(date1.shiftedBy(20030)), 0.0, 1e-5);

    }

	@Test
    public void testShiftedBySeveralTimeShiftSeveralDates() {

		AbsoluteDate date1 = new AbsoluteDate();
		AbsoluteDate date2 = date1.shiftedBy(10000);
        AbsoluteDate[] dates = new AbsoluteDate[] {date1, date2};
        double[] dts = new double[] {10.0, 20.0, 30.0};
	    AbsoluteDateArrayHandling datesForVect = new AbsoluteDateArrayHandling(dates);
	    AbsoluteDate[][] datesShifted = datesForVect.multipleShiftedBy(dts);
	    Assertions.assertEquals(datesShifted[0][0].durationFrom(date1.shiftedBy(10)), 0.0, 1e-5);
	    Assertions.assertEquals(datesShifted[0][1].durationFrom(date1.shiftedBy(20)), 0.0, 1e-5);
	    Assertions.assertEquals(datesShifted[1][1].durationFrom(date2.shiftedBy(20)), 0.0, 1e-5);
	    Assertions.assertEquals(datesShifted[1][2].durationFrom(date2.shiftedBy(30)), 0.0, 1e-5);

    }

	@Test
    public void testDurationFromSeveralDates() {

		AbsoluteDate date1 = new AbsoluteDate();
		AbsoluteDate date2 = date1.shiftedBy(10000);
		AbsoluteDate date3 = date1.shiftedBy(20000);
		AbsoluteDate date4 = date1.shiftedBy(100000);
        AbsoluteDate[] dates = new AbsoluteDate[] {date1, date2, date3};
        AbsoluteDate[] datesComputeDuration = new AbsoluteDate[] {date4, date1};
	    AbsoluteDateArrayHandling datesForVect = new AbsoluteDateArrayHandling(dates);
	    double[][] datesDurations = datesForVect.multipleDurationFrom(datesComputeDuration);
	    Assertions.assertEquals(datesDurations[0][0], date1.durationFrom(date4), 1e-5);
	    Assertions.assertEquals(datesDurations[0][1], date1.durationFrom(date1), 1e-5);
	    Assertions.assertEquals(datesDurations[1][0], date2.durationFrom(date4), 1e-5);
	    Assertions.assertEquals(datesDurations[1][1], date2.durationFrom(date1), 1e-5);
	    Assertions.assertEquals(datesDurations[2][0], date3.durationFrom(date4), 1e-5);
	    Assertions.assertEquals(datesDurations[2][1], date3.durationFrom(date1), 1e-5);

    }

	@Test
    public void testDurationFromCorrespondingDates() {

		AbsoluteDate date1 = new AbsoluteDate();
		AbsoluteDate date2 = date1.shiftedBy(10000);
		AbsoluteDate date3 = date1.shiftedBy(20000);
		AbsoluteDate date4 = date1.shiftedBy(100000);
        AbsoluteDate[] dates = new AbsoluteDate[] {date1, date2, date3};
        AbsoluteDate[] datesComputeDuration = new AbsoluteDate[] {date4, date1, date2};
	    AbsoluteDateArrayHandling datesForVect = new AbsoluteDateArrayHandling(dates);
	    double[] datesDurations = datesForVect.durationFrom(datesComputeDuration);
	    Assertions.assertEquals(datesDurations[0], date1.durationFrom(date4), 1e-5);
	    Assertions.assertEquals(datesDurations[1], date2.durationFrom(date1), 1e-5);
	    Assertions.assertEquals(datesDurations[2], date3.durationFrom(date2), 1e-5);

    }

	@Test
    public void testExceptionDimensions() {

		AbsoluteDate date1 = new AbsoluteDate();
		AbsoluteDate date2 = date1.shiftedBy(10000);
		AbsoluteDate date3 = date1.shiftedBy(20000);
		AbsoluteDate date4 = date1.shiftedBy(100000);
        AbsoluteDate[] dates = new AbsoluteDate[] {date1, date2, date3};
        AbsoluteDate[] datesComputeDuration = new AbsoluteDate[] {date4, date1};
	    AbsoluteDateArrayHandling datesForVect = new AbsoluteDateArrayHandling(dates);
	    try {
	    	datesForVect.durationFrom(datesComputeDuration);
	    	Assertions.fail("an exception should have been thrown");
        } catch (OrekitException oe) {
        	Assertions.assertEquals(LocalizedCoreFormats.DIMENSIONS_MISMATCH, oe.getSpecifier());
        }
    }

}
