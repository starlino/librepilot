/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup UAVOFrSKYSPortBridge UAVO to FrSKY S.PORT Bridge Module
 * @{
 *
 * @file       uavofrskysportbridge.c
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2017-2019
 *             Tau Labs, http://taulabs.org, Copyright (C) 2014
 * @brief      Bridges selected UAVObjects to FrSKY Smart Port bus
 *
 * Since there is no public documentation of SmartPort protocol available,
 * this was put together by studying OpenTx source code, reading
 * tidbits of informations on the Internet and experimenting.
 * @see https://github.com/opentx/opentx/tree/next/radio/src/telemetry
 * @see https://code.google.com/p/telemetry-convert/wiki/FrSkySPortProtocol
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

#include "frsky_packing.h"

#include "pios_board_io.h"

#include "barosensor.h"
#include "flightbatterysettings.h"
#include "flightbatterystate.h"
#include "gpspositionsensor.h"
#include "frskysporttelemetrysettings.h"
#include "hwsettings.h"
#include "taskinfo.h"


#define FRSKY_POLL_REQUEST          0x7e
#define FRSKY_MINIMUM_POLL_INTERVAL 10000

enum frsky_state {
    FRSKY_STATE_WAIT_POLL_REQUEST,
    FRSKY_STATE_WAIT_SENSOR_ID,
    FRSKY_STATE_WAIT_TX_DONE,
};

// Set of objects sent by this module
static const struct frsky_value_item frsky_value_items[] = {
    { FRSKY_GPS_COURSE_ID,  100,  frsky_encode_gps_course, 0 }, // attitude yaw estimate
    { FRSKY_ALT_ID,         100,  frsky_encode_altitude,   0 }, // altitude estimate
    { FRSKY_VARIO_ID,       100,  frsky_encode_vario,      0 }, // vertical speed
    { FRSKY_CURR_ID,        300,  frsky_encode_current,    0 }, // battery current
    { FRSKY_CELLS_ID,       850,  frsky_encode_cells,      0 }, // battery cells 1-2
    { FRSKY_CELLS_ID,       850,  frsky_encode_cells,      1 }, // battery cells 3-4
    { FRSKY_CELLS_ID,       850,  frsky_encode_cells,      2 }, // battery cells 5-6
    { FRSKY_CELLS_ID,       850,  frsky_encode_cells,      3 }, // battery cells 7-8
    { FRSKY_CELLS_ID,       850,  frsky_encode_cells,      4 }, // battery cells 9-10
    { FRSKY_CELLS_ID,       850,  frsky_encode_cells,      5 }, // battery cells 11-12
    { FRSKY_T1_ID,          2000, frsky_encode_t1,         0 }, // baro temperature
    { FRSKY_T2_ID,          1500, frsky_encode_t2,         0 }, // encodes GPS status!
    { FRSKY_FUEL_ID,        600,  frsky_encode_fuel,       0 }, // consumed battery energy
    { FRSKY_ACCX_ID,        100,  frsky_encode_acc,        0 }, // accX
    { FRSKY_ACCY_ID,        100,  frsky_encode_acc,        1 }, // accY
    { FRSKY_ACCZ_ID,        100,  frsky_encode_acc,        2 }, // accZ
    { FRSKY_GPS_LON_LAT_ID, 100,  frsky_encode_gps_coord,  0 }, // lattitude
    { FRSKY_GPS_LON_LAT_ID, 100,  frsky_encode_gps_coord,  1 }, // longitude
    { FRSKY_GPS_ALT_ID,     750,  frsky_encode_gps_alt,    0 }, // gps altitude
    { FRSKY_GPS_SPEED_ID,   200,  frsky_encode_gps_speed,  0 }, // gps speed
    { FRSKY_GPS_TIME_ID,    8000, frsky_encode_gps_time,   0 }, // gps date
    { FRSKY_GPS_TIME_ID,    2000, frsky_encode_gps_time,   1 }, // gps time
    { FRSKY_RPM_ID,         1500, frsky_encode_rpm,        0 }, // encodes flight status!
    { FRSKY_AIR_SPEED_ID,   100,  frsky_encode_airspeed,   0 }, // airspeed
};

struct frsky_sport_telemetry {
    enum frsky_state state;
    int32_t   scheduled_item;
    uint32_t  last_poll_time;
    uintptr_t com;
    bool ignore_echo;
    uint8_t   ignore_rx_chars;
    struct frsky_settings frsky_settings;
    uint16_t  schedule_nr;
    uint32_t  item_last_triggered[NELEMENTS(frsky_value_items)];
    uint16_t  item_schedule_nr[NELEMENTS(frsky_value_items)];
};

static const uint8_t frsky_sensor_ids[] = { 0x1b };

#define FRSKY_SPORT_BAUDRATE 57600

#if defined(PIOS_FRSKY_SPORT_TELEMETRY_STACK_SIZE)
#define STACK_SIZE_BYTES     PIOS_FRSKY_SPORT_TELEMETRY_STACK_SIZE
#else
#define STACK_SIZE_BYTES     672
#endif
#define TASK_PRIORITY        (tskIDLE_PRIORITY)

static struct frsky_sport_telemetry *frsky = 0;
static int32_t uavoFrSKYSPortBridgeInitialize(void);
static void uavoFrSKYSPortBridgeTask(void *parameters);

/**
 * Scan for value item with the longest expired time and schedule it to send in next poll turn
 *
 */

static void frsky_schedule_next_item(void)
{
    frsky->scheduled_item = -1;

    for (uint32_t i = 0; i < NELEMENTS(frsky_value_items); i++) {
        if (frsky_value_items[i].encode_value(&frsky->frsky_settings, 0, true, frsky_value_items[i].fn_arg)) {
            if (frsky->item_schedule_nr[i] == frsky->schedule_nr) {
                continue;
            }
            if (PIOS_DELAY_GetuSSince(frsky->item_last_triggered[i]) > (frsky_value_items[i].period_ms * 1000)) {
                frsky->scheduled_item = i;
                frsky->item_schedule_nr[i] = frsky->schedule_nr;
                break;
            }
        }
    }

    if (frsky->scheduled_item < 0) {
        frsky->schedule_nr++;
    }
}


/**
 * Send value item previously scheduled by frsky_schedule_next_itme()
 * @returns true when item value was sended
 */
static bool frsky_send_scheduled_item(void)
{
    int32_t item = frsky->scheduled_item;

    if ((item >= 0) && (item < (int32_t)NELEMENTS(frsky_value_items))) {
        frsky->item_last_triggered[item] = PIOS_DELAY_GetuS();
        uint32_t value = 0;
        if (frsky_value_items[item].encode_value(&frsky->frsky_settings, &value, false,
                                                 frsky_value_items[item].fn_arg)) {
            frsky->ignore_rx_chars += frsky_send_frame(frsky->com, (uint16_t)(frsky_value_items[item].id), value, false);

            return true;
        }
    }

    return false;
}

/**
 * Process incoming bytes from FrSky S.PORT bus
 * @param[in] b received byte
 */
static void frsky_receive_byte(uint8_t b)
{
    uint32_t i = 0;

    switch (frsky->state) {
    case FRSKY_STATE_WAIT_TX_DONE:
        // because RX and TX are connected, we need to ignore bytes
        // transmited by us
        if (--frsky->ignore_rx_chars == 0) {
            frsky->state = FRSKY_STATE_WAIT_POLL_REQUEST;
        }
        break;

    case FRSKY_STATE_WAIT_POLL_REQUEST:
        if (b == FRSKY_POLL_REQUEST) {
            // X8R is polling us every 13ms
            if (PIOS_DELAY_GetuSSince(frsky->last_poll_time) > FRSKY_MINIMUM_POLL_INTERVAL) {
                frsky->last_poll_time = PIOS_DELAY_GetuS();
                frsky->state = FRSKY_STATE_WAIT_SENSOR_ID;
            }
        }
        break;

    case FRSKY_STATE_WAIT_SENSOR_ID:

        frsky->state = FRSKY_STATE_WAIT_POLL_REQUEST;

        for (i = 0; i < sizeof(frsky_sensor_ids); i++) {
            if (frsky_sensor_ids[i] == b) {
                // get GPSPositionData once here to be more efficient, since
                // GPS position data are very often used by encode() handlers
                if (GPSPositionSensorHandle() != NULL) {
                    GPSPositionSensorGet(&frsky->frsky_settings.gps_position);
                }
                // send item previously scheduled
                frsky_send_scheduled_item();

                if (frsky->ignore_echo && frsky->ignore_rx_chars) {
                    frsky->state = FRSKY_STATE_WAIT_TX_DONE;
                }

                // schedule item for next poll turn
                frsky_schedule_next_item();
                break;
            }
        }
        break;
    }
}

/**
 * Module start routine automatically called after initialization routine
 * @return 0 when was successful
 */
static int32_t uavoFrSKYSPortBridgeStart(void)
{
    if (!frsky) {
        return -1;
    }

    if (FlightBatterySettingsHandle() != NULL
        && FlightBatteryStateHandle() != NULL) {
// TODO: maybe get this setting from somewhere else?
// uint8_t currentPin;
// FlightBatterySettingsCurrentPinGet(&currentPin);
// if (currentPin != FLIGHTBATTERYSETTINGS_CURRENTPIN_NONE)
        frsky->frsky_settings.use_current_sensor = true;
        FlightBatterySettingsGet(&frsky->frsky_settings.battery_settings);
        frsky->frsky_settings.batt_cell_count    = frsky->frsky_settings.battery_settings.NbCells;
    }
// This is just to check if barometer is enabled.
// if (BaroSensorHandle() != NULL
// && PIOS_SENSORS_GetQueue(PIOS_SENSOR_BARO) != NULL)
    frsky->frsky_settings.use_baro_sensor = true;


    xTaskHandle task;
    xTaskCreate(uavoFrSKYSPortBridgeTask, "FrSky SPort Telemetry", STACK_SIZE_BYTES / 4, NULL, TASK_PRIORITY, &task);
    PIOS_TASK_MONITOR_RegisterTask(TASKINFO_RUNNING_FRSKYSPORTTELEMETRY, task);

    return 0;
}

static void FrSKYSPortTelemetrySettingsUpdatedCb(__attribute__((unused)) UAVObjEvent *ev)
{
    FrSKYSPortTelemetrySettingsData settings;

    FrSKYSPortTelemetrySettingsGet(&settings);
}


/**
 * Module initialization routine
 * @return 0 when initialization was successful
 */
static int32_t uavoFrSKYSPortBridgeInitialize(void)
{
    FrSKYSPortTelemetrySettingsInitialize();

    if (PIOS_COM_FRSKY_SPORT) {
        frsky = pios_malloc(sizeof(struct frsky_sport_telemetry));

        if (frsky != NULL) {
            memset(frsky, 0x00, sizeof(struct frsky_sport_telemetry));

            frsky->frsky_settings.use_current_sensor = false;
            frsky->frsky_settings.batt_cell_count    = 0;
            frsky->frsky_settings.use_baro_sensor    = false;
            frsky->state = FRSKY_STATE_WAIT_POLL_REQUEST;
            frsky->last_poll_time = PIOS_DELAY_GetuS();
            frsky->scheduled_item = -1;
            frsky->com   = PIOS_COM_FRSKY_SPORT;
            frsky->ignore_echo = true; // This has to be true when RX & TX hw serial lines are connected. Otherwise false. (enforced below by setting half duplex. this makes internal connection between rx and tx
            // connect TX pin of flight controller to UNINVERTED SPort
            // (F4 based controllers do not have TX inversion capability.
            // Use external inverter or solder to uninverted receiver pin)
            // TODO: Add code/PIOS driver to enable inversion for F3 based convertes
            // TODO: implement FPORT driver
            frsky->schedule_nr = 1;

            uint8_t i;
            for (i = 0; i < NELEMENTS(frsky_value_items); i++) {
                frsky->item_last_triggered[i] = PIOS_DELAY_GetuS();
            }

            // Set Port options:
            // BAUD rate
            PIOS_COM_ChangeBaud(frsky->com, FRSKY_SPORT_BAUDRATE);

            HwSettingsSPortModeOptions options;
            HwSettingsSPortModeGet(&options);
            bool halfduplex;
            enum PIOS_USART_Inverted inverted;
            switch (options) {
            case HWSETTINGS_SPORTMODE_HALFDUPLEXNONINVERTED:
                halfduplex = true;
                inverted   = PIOS_USART_Inverted_None;
                break;
            case HWSETTINGS_SPORTMODE_HALFDUPLEXINVERTED:
                halfduplex = true;
                inverted   = PIOS_USART_Inverted_RxTx;
                break;
            case HWSETTINGS_SPORTMODE_FULLDUPLEXNONINVERTED:
                halfduplex = false;
                inverted   = PIOS_USART_Inverted_None;
                break;
            case HWSETTINGS_SPORTMODE_FULLDUPLEXINVERTED:
                halfduplex = false;
                inverted   = PIOS_USART_Inverted_RxTx;
                break;
            }

            // Port Inversion (Not available on STM32F4, will have no effect)
            PIOS_COM_Ioctl(frsky->com, PIOS_IOCTL_USART_SET_INVERTED, &inverted);

            // HalfDplex mode (Not available on STM32F0, will have no effect)
            PIOS_COM_Ioctl(frsky->com, PIOS_IOCTL_USART_SET_HALFDUPLEX, &halfduplex);

            FrSKYSPortTelemetrySettingsConnectCallback(FrSKYSPortTelemetrySettingsUpdatedCb);

            FrSKYSPortTelemetrySettingsUpdatedCb(0);

            return 0;
        }
    }

    return -1;
}
MODULE_INITCALL(uavoFrSKYSPortBridgeInitialize, uavoFrSKYSPortBridgeStart);

/**
 * Main task routine
 * @param[in] parameters parameter given by PIOS_Thread_Create()
 */
static void uavoFrSKYSPortBridgeTask(__attribute__((unused)) void *parameters)
{
    while (1) {
        uint8_t b = 0;
        uint16_t count = PIOS_COM_ReceiveBuffer(frsky->com, &b, 1, 0xffffffff);
        if (count) {
            frsky_receive_byte(b);
        }
    }
}

/**
 * @}
 * @}
 */
