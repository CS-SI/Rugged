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

import org.hipparchus.exception.LocalizedCoreFormats;
import org.orekit.errors.OrekitException;
import org.orekit.time.AbsoluteDate;

/** AbsoluteDateArrayHandling consist of additions to AbsoluteDate to handle arrays.
 * @author Melina Vanel
 */
public class AbsoluteDateArrayHandling {

    /** Dates array on which we want to apply time shift or compute duration. */
    private final AbsoluteDate[] dates;

    /** Simple constructor.
     * @param dates is an array of absolute dates on which we want to apply time shift or
     * compute duration
     */
    public AbsoluteDateArrayHandling(final AbsoluteDate[] dates) {
        this.dates = dates.clone();
    }

    /** Get instance dates array.
     * @return dates array
     */
    public AbsoluteDate[] getDates() {
        return this.dates.clone();
    }

    /** Get time-shifted dates for several dates or several time shifts.
     * If instance dates = [date1, date2, ..., daten] and argument
     * dts = [dts1, dts2, ..., dtsn] then this function will return a matrix
     * [[date1 shiftedby dts1, date1 shiftedBy dts2, ..., date1 shiftedBy dtsn],
     * [date2 shiftedby dts1, date2 shiftedBy dts2, ..., date2 shiftedBy dtsn],
     * [...]
     * [daten shiftedby dts1, daten shiftedBy dts2, ..., date1 shiftedBy dtsn]].
     * If ones want to apply only 1 time shift corresponding to 1 date see
     * {@link #shiftedBy(double[])}.
     * @param dts time shifts array in seconds we want to apply to dates
     * @return a matrix of new dates, shifted with respect to wanted time
     * shifts. If instance dates = [date1, date2, ..., daten] each line
     * correspond to one date (for example date1 shiftedBy all timeshifts
     * (building the different columns))
     */
    public AbsoluteDate[][] multipleShiftedBy(final double[] dts) {

        final AbsoluteDate[][] datesShifted = new AbsoluteDate[dates.length][dts.length];
        int index_dates = 0;

        for (AbsoluteDate date: this.dates) {
            final AbsoluteDate[] dateShifted = new AbsoluteDate[dts.length];
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

    /** Get time-shifted dates for several dates and corresponding time shifts.
     * If instance dates = [date1, date2, ..., daten] and argument
     * dts = [dts1, dts2, ..., dtsn] then this function will return
     * [date1 shiftedby dts1, date2 shiftedBy dts2, ..., daten shiftedBy dtsn]. If
     * several time shift want to be applied on each date see
     * {@link #multipleShiftedBy(double[])}.
     * @param dts time shifts array in seconds we want to apply to corresponding dates.
     * Warning, must be same length as dates.
     * @return an 1D array of new dates, shifted with respect to wanted corresponding time
     * shifts.
     */
    public AbsoluteDate[] shiftedBy(final double[] dts) {

        // Check same dimensions
        if (dates.length != dts.length) {
            throw new OrekitException(LocalizedCoreFormats.DIMENSIONS_MISMATCH,
                                      dates.length, dts.length);
        }

        final AbsoluteDate[] datesShifted = new AbsoluteDate[dates.length];
        int index_dates = 0;

        for (AbsoluteDate date: this.dates) {
            datesShifted[index_dates] = date.shiftedBy(dts[index_dates]);
            index_dates += 1;

        }
        return datesShifted;
    }

    /** Get array with durations between instances dates and given dates
     * If instance dates = [date1, date2, ..., daten] and argument
     * datesForDuration = [d1, d2, ..., dn] then this function will return a matrix
     * [[date1 durationFrom d1, date1 durationFrom d2, ..., date1 durationFrom dn],
     * [date2 durationFrom d1, date2 durationFrom d2, ..., date2 durationFrom dn],
     * [...]
     * [daten durationFrom d1, daten durationFrom d2, ..., date1 durationFrom dn]].
     * If ones want to compute duration from only 1 date corresponding to 1 instance date see
     * {@link #durationFrom(AbsoluteDate[])}.
     * @param datesForDuration dates for which we want to compute the duration form instances dates
     * @return a matrix of double representing durations from instance dates
     * If instance dates = [date1, date2, ..., daten] each line
     * correspond to one date (for example date1 duration from all given dates in arguments
     * (building the different columns))
     */
    public double[][] multipleDurationFrom(final AbsoluteDate[] datesForDuration) {

        final double[][] durationsFromDates = new double[dates.length][datesForDuration.length];
        int index_dates = 0;

        for (AbsoluteDate date: this.dates) {
            final double[] durationFromDate = new double[datesForDuration.length];
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

    /** Get array with durations between instances dates and corresponding given dates
     * If instance dates = [date1, date2, ..., daten] and argument
     * datesForDuration = [d1, d2, ..., dn] then this function will return
     * [date1 durationFrom d1, date2 durationFrom d2, ..., daten durationFrom dn]. If
     * duration from from all arguments dates wants to be compute on each date see
     * {@link #multipleDurationFrom(AbsoluteDate[])}.
     * @param datesForDuration dates for which we want to compute the duration form instances dates.
     * Warning must have same length as instance dates.
     * @return a array of double representing durations between instance dates and corresponding
     * argument dates
     */
    public double[] durationFrom(final AbsoluteDate[] datesForDuration) {

        // Check same dimensions
        if (dates.length != datesForDuration.length) {
            throw new OrekitException(LocalizedCoreFormats.DIMENSIONS_MISMATCH,
                                      dates.length, datesForDuration.length);
        }

        final double[] durationsFromDates = new double[dates.length];
        int index_dates = 0;

        for (AbsoluteDate date: this.dates) {
            durationsFromDates[index_dates] = date.durationFrom(datesForDuration[index_dates]);
            index_dates += 1;

        }
        return durationsFromDates;
    }

}
