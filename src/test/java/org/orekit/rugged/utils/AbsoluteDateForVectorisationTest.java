/* Copyright 2013-2022 CS GROUP
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


import org.junit.Assert;
import org.junit.Test;
import org.orekit.time.AbsoluteDate;

public class AbsoluteDateForVectorisationTest {
	@Test
    public void testShiftedBySeveralTimeShiftOneDate() {

		AbsoluteDate date1 = new AbsoluteDate();
        AbsoluteDate[] dates = new AbsoluteDate[] {date1};
        double[] dts = new double[] {10.0, 20.0, 30.0};
	    AbsoluteDateArrayHandling datesForVect = new AbsoluteDateArrayHandling(dates);
	    AbsoluteDate[][] datesShifted = datesForVect.shiftedBySeveralTimeShift(dts);
	    Assert.assertEquals(datesShifted[0][0].durationFrom(date1.shiftedBy(10)), 0.0, 1e-5);
	    Assert.assertEquals(datesShifted[0][1].durationFrom(date1.shiftedBy(20)), 0.0, 1e-5);

    }

	@Test
    public void testShiftedBySeveralTimeShiftSeveralDates() {

		AbsoluteDate date1 = new AbsoluteDate();
		AbsoluteDate date2 = date1.shiftedBy(10000);
        AbsoluteDate[] dates = new AbsoluteDate[] {date1, date2};
        double[] dts = new double[] {10.0, 20.0, 30.0};
	    AbsoluteDateArrayHandling datesForVect = new AbsoluteDateArrayHandling(dates);
	    AbsoluteDate[][] datesShifted = datesForVect.shiftedBySeveralTimeShift(dts);
	    Assert.assertEquals(datesShifted[0][0].durationFrom(date1.shiftedBy(10)), 0.0, 1e-5);
	    Assert.assertEquals(datesShifted[0][1].durationFrom(date1.shiftedBy(20)), 0.0, 1e-5);
	    Assert.assertEquals(datesShifted[1][1].durationFrom(date2.shiftedBy(20)), 0.0, 1e-5);
	    Assert.assertEquals(datesShifted[1][2].durationFrom(date2.shiftedBy(30)), 0.0, 1e-5);

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
	    double[][] datesDurations = datesForVect.durationsFromSeveralTimeShifts(datesComputeDuration);
	    Assert.assertEquals(datesDurations[0][0], date1.durationFrom(date4), 1e-5);
	    Assert.assertEquals(datesDurations[0][1], date1.durationFrom(date1), 1e-5);
	    Assert.assertEquals(datesDurations[1][0], date2.durationFrom(date4), 1e-5);
	    Assert.assertEquals(datesDurations[1][1], date2.durationFrom(date1), 1e-5);
	    Assert.assertEquals(datesDurations[2][0], date3.durationFrom(date4), 1e-5);
	    Assert.assertEquals(datesDurations[2][1], date3.durationFrom(date1), 1e-5);

    }


}
