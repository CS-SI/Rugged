/* Copyright 2013-2017 CS Systèmes d'Information
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
package org.orekit.rugged.api;


/** Enumerate for cartographic projection type.
 * @author Lucie Labat-Allee
 * @since 2.0
 */
public enum ProjectionTypeId {

	/** Constant for WGS 84 projection type. */
	WGS84,

	/** Constant for UTM zone 1N projection type. */
	UTM1N,

	/** Constant for UTM zone 60N projection type. */
	UTM60N,

	/** Constant for UTM zone 1S projection type. */
	UTM1S,

	/** Constant for UTM zone 60S projection type. */
	UTM60S,

	/** Constant for Lambert-93 projection type. */
	LAMBERT93
}
