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

import org.orekit.time.AbsoluteDate;

/** AbsoluteDateForVectorisation consist of additions to AbsoluteDate to handle arrays for vectorization
 * @author Melina Vanel
 */
public class AbsoluteDateArrayHandling {
    
    /** Dates array on which we want to apply time shift or compute duration */
    public AbsoluteDate[] dates;

    /** Simple constructor.
     * @param dates is an array of absolute dates on which we want to apply time shift or
     * compute duration
     */
    public AbsoluteDateArrayHandling(final AbsoluteDate[] dates) {
        this.dates = dates.clone();
    }
    
    /** Get time-shifted dates for several dates or several time shifts.
     * @param dts time shifts array in seconds we want to apply to dates
     * @return a matrix of new dates, shifted with respect to wanted time
     * shifts. If instance dates = [date1, date2, ..., daten] each line
     * correspond to one date (for example date1 shiftedBy all timeshifts
     * (building the different columns))
     */
    public AbsoluteDate[][] shiftedBySeveralTimeShift(final double[] dts) {

        AbsoluteDate[][] datesShifted = new AbsoluteDate[dates.length][dts.length];
        int index_dates = 0;

        for (AbsoluteDate date: this.dates) {
            AbsoluteDate[] dateShifted = new AbsoluteDate[dts.length];
            int index_dts = 0;
            for (double dt: dts) {
                dateShifted[index_dts] = date.shiftedBy(dt);
                index_dts += 1;
            }
            datesShifted[index_dates] = dateShifted;
            index_dates += 1;

        }
        return (AbsoluteDate[][]) datesShifted;
    }

    /** Get array with durations between instances dates and given dates
     * @param datesForDuration dates for which we want to compute the duration form instances dates
     * @return a matrix of double representing durations from instance dates 
     * If instance dates = [date1, date2, ..., daten] each line
     * correspond to one date (for example date1 duration from all given dates in arguments
     * (building the different columns))
     */
    public double[][] durationsFromSeveralTimeShifts(final AbsoluteDate[] datesForDuration) {

        double[][] durationsFromDates = new double[dates.length][datesForDuration.length];
        int index_dates = 0;

        for (AbsoluteDate date: this.dates) {
            double[] durationFromDate = new double[datesForDuration.length];
            int index_datesForDuration = 0;
            for (AbsoluteDate dateForDuration: datesForDuration) {
            	durationFromDate[index_datesForDuration] = date.durationFrom(dateForDuration);
                index_datesForDuration += 1;
            }
            durationsFromDates[index_dates] = durationFromDate;
            index_dates += 1;

        }
        return (double[][]) durationsFromDates;
    }

    
}
