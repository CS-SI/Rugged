/* Copyright 2013-2014 CS Systèmes d'Information
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
package org.orekit.rugged.geotiff;


enum GeographicCoordinateSystemType {

    GCS_UNDEFINED(0),
    GCS_ADINDAN(4201),
    GCS_AGD66(4202),
    GCS_AGD84(4203),
    GCS_AIN_EL_ABD(4204),
    GCS_AFGOOYE(4205),
    GCS_AGADEZ(4206),
    GCS_LISBON(4207),
    GCS_ARATU(4208),
    GCS_ARC_1950(4209),
    GCS_ARC_1960(4210),
    GCS_BATAVIA(4211),
    GCS_BARBADOS(4212),
    GCS_BEDUARAM(4213),
    GCS_BEIJING_1954(4214),
    GCS_BELGE_1950(4215),
    GCS_BERMUDA_1957(4216),
    GCS_BERN_1898(4217),
    GCS_BOGOTA(4218),
    GCS_BUKIT_RIMPAH(4219),
    GCS_CAMACUPA(4220),
    GCS_CAMPO_INCHAUSPE(4221),
    GCS_CAPE(4222),
    GCS_CARTHAGE(4223),
    GCS_CHUA(4224),
    GCS_CORREGO_ALEGRE(4225),
    GCS_COTE_D_IVOIRE(4226),
    GCS_DEIR_EZ_ZOR(4227),
    GCS_DOUALA(4228),
    GCS_EGYPT_1907(4229),
    GCS_ED50(4230),
    GCS_ED87(4231),
    GCS_FAHUD(4232),
    GCS_GANDAJIKA_1970(4233),
    GCS_GAROUA(4234),
    GCS_GUYANE_FRANCAISE(4235),
    GCS_HU_TZU_SHAN(4236),
    GCS_HD72(4237),
    GCS_ID74(4238),
    GCS_INDIAN_1954(4239),
    GCS_INDIAN_1975(4240),
    GCS_JAMAICA_1875(4241),
    GCS_JAD69(4242),
    GCS_KALIANPUR(4243),
    GCS_KANDAWALA(4244),
    GCS_KERTAU(4245),
    GCS_KOC(4246),
    GCS_LA_CANOA(4247),
    GCS_PSAD56(4248),
    GCS_LAKE(4249),
    GCS_LEIGON(4250),
    GCS_LIBERIA_1964(4251),
    GCS_LOME(4252),
    GCS_LUZON_1911(4253),
    GCS_HITO_XVIII_1963(4254),
    GCS_HERAT_NORTH(4255),
    GCS_MAHE_1971(4256),
    GCS_MAKASSAR(4257),
    GCS_EUREF89(4258),
    GCS_MALONGO_1987(4259),
    GCS_MANOCA(4260),
    GCS_MERCHICH(4261),
    GCS_MASSAWA(4262),
    GCS_MINNA(4263),
    GCS_MHAST(4264),
    GCS_MONTE_MARIO(4265),
    GCS_M_PORALOKO(4266),
    GCS_NAD27(4267),
    GCS_NAD_MICHIGAN(4268),
    GCS_NAD83(4269),
    GCS_NAHRWAN_1967(4270),
    GCS_NAPARIMA_1972(4271),
    GCS_GD49(4272),
    GCS_NGO_1948(4273),
    GCS_DATUM_73(4274),
    GCS_NTF(4275),
    GCS_NSWC_9Z_2(4276),
    GCS_OSGB_1936(4277),
    GCS_OSGB70(4278),
    GCS_OS_SN80(4279),
    GCS_PADANG(4280),
    GCS_PALESTINE_1923(4281),
    GCS_POINTE_NOIRE(4282),
    GCS_GDA94(4283),
    GCS_PULKOVO_1942(4284),
    GCS_QATAR(4285),
    GCS_QATAR_1948(4286),
    GCS_QORNOQ(4287),
    GCS_LOMA_QUINTANA(4288),
    GCS_AMERSFOORT(4289),
    GCS_RT38(4290),
    GCS_SAD69(4291),
    GCS_SAPPER_HILL_1943(4292),
    GCS_SCHWARZECK(4293),
    GCS_SEGORA(4294),
    GCS_SERINDUNG(4295),
    GCS_SUDAN(4296),
    GCS_TANANARIVE(4297),
    GCS_TIMBALAI_1948(4298),
    GCS_TM65(4299),
    GCS_TM75(4300),
    GCS_TOKYO(4301),
    GCS_TRINIDAD_1903(4302),
    GCS_TC_1948(4303),
    GCS_VOIROL_1875(4304),
    GCS_VOIROL_UNIFIE(4305),
    GCS_BERN_1938(4306),
    GCS_NORD_SAHARA_1959(4307),
    GCS_STOCKHOLM_1938(4308),
    GCS_YACARE(4309),
    GCS_YOFF(4310),
    GCS_ZANDERIJ(4311),
    GCS_MGI(4312),
    GCS_BELGE_1972(4313),
    GCS_DHDN(4314),
    GCS_CONAKRY_1905(4315),
    GCS_WGS_72(4322),
    GCS_WGS_72BE(4324),
    GCS_WGS_84(4326),
    GCS_BERN_1898_BERN(4801),
    GCS_BOGOTA_BOGOTA(4802),
    GCS_LISBON_LISBON(4803),
    GCS_MAKASSAR_JAKARTA(4804),
    GCS_MGI_FERRO(4805),
    GCS_MONTE_MARIO_ROME(4806),
    GCS_NTF_PARIS(4807),
    GCS_PADANG_JAKARTA(4808),
    GCS_BELGE_1950_BRUSSELS(4809),
    GCS_TANANARIVE_PARIS(4810),
    GCS_VOIROL_1875_PARIS(4811),
    GCS_VOIROL_UNIFIE_PARIS(4812),
    GCS_BATAVIA_JAKARTA(4813),
    GCS_ATF_PARIS(4901),
    GCS_NDG_PARIS(4902),
    GCSE_AIRY1830(4001),
    GCSE_AIRYMODIFIED1849(4002),
    GCSE_AUSTRALIANNATIONALSPHEROID(4003),
    GCSE_BESSEL1841(4004),
    GCSE_BESSELMODIFIED(4005),
    GCSE_BESSELNAMIBIA(4006),
    GCSE_CLARKE1858(4007),
    GCSE_CLARKE1866(4008),
    GCSE_CLARKE1866MICHIGAN(4009),
    GCSE_CLARKE1880_BENOIT(4010),
    GCSE_CLARKE1880_IGN(4011),
    GCSE_CLARKE1880_RGS(4012),
    GCSE_CLARKE1880_ARC(4013),
    GCSE_CLARKE1880_SGA1922(4014),
    GCSE_EVEREST1830_1937ADJUSTMENT(4015),
    GCSE_EVEREST1830_1967DEFINITION(4016),
    GCSE_EVEREST1830_1975DEFINITION(4017),
    GCSE_EVEREST1830MODIFIED(4018),
    GCSE_GRS1980(4019),
    GCSE_HELMERT1906(4020),
    GCSE_INDONESIANNATIONALSPHEROID(4021),
    GCSE_INTERNATIONAL1924(4022),
    GCSE_INTERNATIONAL1967(4023),
    GCSE_KRASSOWSKY1940(4024),
    GCSE_NWL9D(4025),
    GCSE_NWL10D(4026),
    GCSE_PLESSIS1817(4027),
    GCSE_STRUVE1860(4028),
    GCSE_WAROFFICE(4029),
    GCSE_WGS84(4030),
    GCSE_GEM10C(4031),
    GCSE_OSU86F(4032),
    GCSE_OSU91A(4033),
    GCSE_CLARKE1880(4034),
    GCSE_SPHERE(4035);

    /** Type ID. */
    private final int id;

    /** Simple constructor.
     * @param id key id
     */
    private GeographicCoordinateSystemType(final int id) {
        this.id = id;
    }

    /** Get the type corresponding to an id.
     * @param id type id
     * @return the type corresponding to the id
     * @exception IllegalArgumentException if the id does not correspond to a known type
     */
    public static GeographicCoordinateSystemType getType(final int id) {
        for (GeographicCoordinateSystemType type : values()) {
            if (type.id == id) {
                return type;
            }
        }
        throw new IllegalArgumentException();
    }

}
