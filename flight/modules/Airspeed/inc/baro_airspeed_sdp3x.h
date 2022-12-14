/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup AirspeedModule Airspeed Module
 * @brief Calculate airspeed as a function of the difference between sequential GPS velocity and attitude measurements
 * @{
 *
 * @file       baro_airspeed_etasv3.h
 * @author     The LibrePilot Team, http://www.librepilot.org Copyright (C) 2021.
 * @brief      Airspeed module, reads temperature and pressure from SDP3X
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#ifndef BARO_AIRSPEED_SDP3X_H
#define BARO_AIRSPEED_SDP3X_H
#if defined(PIOS_INCLUDE_SDP3X)

void baro_airspeedGetSDP3X(AirspeedSensorData *airspeedSensor, AirspeedSettingsData *airspeedSettings);

#endif
#endif // BARO_AIRSPEED_SDP3X_H

/**
 * @}
 * @}
 */
