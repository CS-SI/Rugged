/* Copyright 2013-2016 CS Systèmes d'Information
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
package org.orekit.rugged.linesensor;

import org.orekit.time.AbsoluteDate;


/** Linear model for {@link LineDatation line datation}.
 * <p>
 * Instances of this class are guaranteed to be immutable.
 * </p>
 * @author Luc Maisonobe
 * @author Guylaine Prat

 */
public class LinearLineDatation implements LineDatation {

    /** Reference date. */
    private final AbsoluteDate referenceDate;

    /** Line number at reference date. */
    private final double referenceLine;

    /** Rate of lines scanning (lines / seconds). */
    private final double rate;

    /** Simple constructor.
     * @param referenceDate reference date
     * @param referenceLine line number at reference date
     * @param rate rate of lines scanning (lines / seconds)
     */
    public LinearLineDatation(final AbsoluteDate referenceDate, final double referenceLine,
                              final double rate) {
        this.referenceDate = referenceDate;
        this.referenceLine = referenceLine;
        this.rate          = rate;
    }

    /** {@inheritDoc} */
    @Override
    public AbsoluteDate getDate(final double lineNumber) {
        return referenceDate.shiftedBy((lineNumber - referenceLine) / rate);
    }

    /** {@inheritDoc} */
    @Override
    public double getLine(final AbsoluteDate date) {
        return referenceLine + rate * date.durationFrom(referenceDate);
    }

    /** {@inheritDoc} */
    @Override
    public double getRate(final double lineNumber) {
        return rate;
    }

}
