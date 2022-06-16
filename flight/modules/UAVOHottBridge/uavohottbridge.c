/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup UAVOHoTTBridge UAVO to HoTT Bridge Telemetry Module
 * @{
 *
 * @file       uavohottbridge.c
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2017-2019.
 *             Tau Labs, http://taulabs.org, Copyright (C) 2013-2014
 * @brief      sends telemery data on HoTT request
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
 *
 * Additional note on redistribution: The copyright and license notices above
 * must be maintained in each individual source file that is a derivative work
 * of this source file; otherwise redistribution is prohibited.
 */

#include "openpilot.h"
#include "hwsettings.h"
#include "taskinfo.h"
#include "callbackinfo.h"
#include "hottbridgesettings.h"
#include "attitudestate.h"
#include "accelstate.h"
#include "barosensor.h"
#include "flightbatterystate.h"
#include "flightstatus.h"
#include "gyrosensor.h"
#include "gpspositionsensor.h"
#include "gpstime.h"
#include "airspeedstate.h"
#include "homelocation.h"
#include "positionstate.h"
#include "systemalarms.h"
#include "velocitystate.h"
#include "temperaturestate.h"
#include "hottbridgestatus.h"
#include "hottbridgesettings.h"
#include "flightbatterysettings.h"
#include "hwsettings.h"
#include "revosettings.h"
#include "attitudesettings.h"
#include "gpssettings.h"
#include "homelocation.h"
#include "objectpersistence.h"
#include "pios_sensors.h"
#include "uavohottbridge.h"

#include "pios_board_io.h"

#if defined(PIOS_INCLUDE_HOTT_BRIDGE)

#if defined(PIOS_HoTT_STACK_SIZE)
#define STACK_SIZE_BYTES    PIOS_HoTT_STACK_SIZE
#else
#define STACK_SIZE_BYTES    2048
#endif
#define TASK_PRIORITY       CALLBACK_TASK_AUXILIARY

#define ADC_XX_PIN_NOTFOUND -1

#define maxu2(x) ((x) > 99 ? 99 : (x))
#define maxu3(x) ((x) > 999 ? 999 : (x))
#define maxu4(x) ((x) > 9999 ? 9999 : (x))

#define maxd2(x) ((x) < -99 ? -99 : (maxu2(x)))
#define maxd3(x) ((x) < -999 ? -999 : (maxu3(x)))
#define maxd4(x) ((x) < -9999 ? -9999 : (maxu4(x)))

static bool module_enabled = false;

// Private variables
static bool module_enabled;
static struct telemetrydata *telestate;
static HoTTBridgeStatusData status;
static HomeLocationSetOptions homeSetFlash;

// Private functions
static void uavoHoTTBridgeTask(void *parameters);
static uint16_t build_VARIO_message(struct hott_vario_message *msg);
static uint16_t build_GPS_message(struct hott_gps_message *msg);
static uint16_t build_GAM_message(struct hott_gam_message *msg);
static uint16_t build_EAM_message(struct hott_eam_message *msg);
static uint16_t build_ESC_message(struct hott_esc_message *msg);
static uint8_t build_TEXT_message(struct hott_text_message *msg, uint8_t page, uint8_t current_line, int8_t value_change, uint8_t step_change, bool edit_line, bool exit_menu);
static char *reverse_pixels(char *line, uint8_t from_char, uint8_t to_char);
static uint8_t get_page(uint8_t page, bool next);
static uint8_t enable_disable_warning(uint8_t value);
static uint8_t enable_disable_sensor(uint8_t value);
static float get_redirect_sensor_value(uint8_t hott_sensor);
static int16_t get_new_value(int16_t current_value, int8_t value_change, uint8_t step, int16_t min, int16_t max);
static int8_t get_newADCPin_value(uint8_t *adcRouting, int8_t from_pin, int8_t value_change);
static uint8_t calc_checksum(uint8_t *data, uint16_t size);
static uint8_t generate_warning();
static void store_settings(uint8_t page, uint8_t current_line);
static void update_telemetrydata();
static void convert_long2gps(int32_t value, uint8_t *dir, uword_t *min, uword_t *sec);
static uint8_t scale_float2uint8(float value, float scale, float offset);
static int8_t scale_float2int8(float value, float scale, float offset);
static uword_t scale_float2uword(float value, float scale, float offset);

/**
 * Module start routine automatically called after initialization routine
 * @return 0 when was successful
 */
static int32_t uavoHoTTBridgeStart(void)
{
    status.TxPackets = 0;
    status.RxPackets = 0;
    status.TxFail    = 0;
    status.RxFail    = 0;


    // Start task
    if (module_enabled) {
        xTaskHandle taskHandle;

        xTaskCreate(uavoHoTTBridgeTask, "uavoHoTTBridge", STACK_SIZE_BYTES / 4, NULL, TASK_PRIORITY, &taskHandle);
        PIOS_TASK_MONITOR_RegisterTask(TASKINFO_RUNNING_UAVOHOTTBRIDGE, taskHandle);
    }

    return 0;
}

/**
 * Module initialization routine
 * @return 0 when initialization was successful
 */
static int32_t uavoHoTTBridgeInitialize(void)
{
    if (PIOS_COM_HOTT) {
        module_enabled = true;
        // HoTT telemetry baudrate is fixed to 19200

        PIOS_COM_ChangeBaud(PIOS_COM_HOTT, 19200);
        bool param = true;
        PIOS_COM_Ioctl(PIOS_COM_HOTT, PIOS_IOCTL_USART_SET_HALFDUPLEX, &param);
        HoTTBridgeStatusInitialize();

        // allocate memory for telemetry data
        telestate = (struct telemetrydata *)pios_malloc(sizeof(*telestate));

        if (telestate == NULL) {
            // there is not enough free memory. the module could not run.
            module_enabled = false;
            return -1;
        }
    } else {
        module_enabled = false;
    }
    return 0;
}
MODULE_INITCALL(uavoHoTTBridgeInitialize, uavoHoTTBridgeStart);

/**
 * Main task. It does not return.
 */
static void uavoHoTTBridgeTask(__attribute__((unused)) void *parameters)
{
    uint8_t rx_buffer[2];
    uint8_t tx_buffer[HOTT_MAX_MESSAGE_LENGTH];
    uint16_t message_size;

    // clear all state values
    memset(telestate, 0, sizeof(*telestate));

    // init minimal values
    telestate->min_voltage = 100.0f;

    // initialize timer variables
    portTickType lastSysTime = xTaskGetTickCount();
    // idle delay between telemetry request and answer
    uint32_t idledelay = IDLE_TIME;
    // data delay between transmitted bytes
    uint32_t datadelay = DATA_TIME;

    // Get stored homeSet status at start
    if (HomeLocationHandle() != NULL) {
        UAVObjLoad(HomeLocationHandle(), 0); // load from flash
        HomeLocationSetGet(&homeSetFlash);
    }

    // work on hott telemetry. endless loop.
    while (1) {
        // clear message size on every loop before processing
        message_size = 0;

        // shift receiver buffer. make room for one byte.
        rx_buffer[1] = rx_buffer[0];

        // wait for a byte of telemetry request in data delay interval
        while (PIOS_COM_ReceiveBuffer(PIOS_COM_HOTT, rx_buffer, 1, 0) == 0) {
            vTaskDelayUntil(&lastSysTime, datadelay / portTICK_RATE_MS);
        }
        // set start trigger point
        lastSysTime = xTaskGetTickCount();

        // examine received data stream
        if (rx_buffer[1] == HOTT_BINARY_ID) {
            // first received byte looks like a binary request. check second received byte for a sensor id.
            switch (rx_buffer[0]) {
            case HOTT_VARIO_ID:
                message_size = build_VARIO_message((struct hott_vario_message *)tx_buffer);
                break;
            case HOTT_GPS_ID:
                message_size = build_GPS_message((struct hott_gps_message *)tx_buffer);
                break;
            case HOTT_GAM_ID:
                message_size = build_GAM_message((struct hott_gam_message *)tx_buffer);
                break;
            case HOTT_EAM_ID:
                message_size = build_EAM_message((struct hott_eam_message *)tx_buffer);
                break;
            case HOTT_ESC_ID:
                message_size = build_ESC_message((struct hott_esc_message *)tx_buffer);
                break;
            default:
                message_size = 0;
            }
        } else if (rx_buffer[1] == HOTT_TEXT_ID) {
            // first received byte looks like a text request. check second received byte for a valid button.
            uint8_t id_key        = rx_buffer[0] & 0x0F;
            uint8_t id_sensor     = rx_buffer[0] >> 4;

            static bool edit_line = false;
            bool exit_menu        = false;
            int8_t value_change   = 0;
            static uint8_t step_change = 0;

            // define allowed edited lines for Main, Main Config, GPS config, Battery config, VarioWarnings, VarioLimits, GPS, General, Electric, Esc pages and Sensor redirect page
            uint8_t min_line[]  = { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 };
            uint8_t max_line[]  = { 6, 5, 5, 6, 8, 8, 7, 7, 7, 7, 8 };

            static uint8_t page = HOTTTEXT_PAGE_MAIN;
            static uint8_t edit_status  = HOTTTEXT_EDITSTATUS_DONE;
            static uint8_t last_page    = 0;
            static uint8_t current_line = 0;

            // menu keys
            if (edit_line) {
                switch (id_key) {
                case HOTT_KEY_DEC: // up
                    value_change--;
                    break;
                case HOTT_KEY_INC: // down
                    value_change++;
                    break;
                case HOTT_KEY_PREV: // left
                    if (step_change < HOTTTEXT_EDITSTATUS_STEP10K) {
                        step_change++;
                    }
                    break;
                case HOTT_KEY_NEXT: // right
                    if (step_change > HOTTTEXT_EDITSTATUS_STEP1) {
                        step_change--;
                    }
                    break;
                case HOTT_KEY_SET: // Set
                    value_change = 0;
                    step_change  = 0;
                    if (edit_status != HOTTTEXT_EDITSTATUS_DONE) {
                        store_settings(page, current_line);
                        edit_line = false; // exit edit mode
                    }
                    break;
                default:
                    value_change = 0;
                }
            } else {
                switch (id_key) {
                case HOTT_KEY_DEC: // up
                    current_line--;
                    break;
                case HOTT_KEY_INC: // down
                    current_line++;
                    break;
                case HOTT_KEY_PREV: // left
                    exit_menu = (page == HOTTTEXT_PAGE_MAIN) ? true : false;
                    page = get_page(page, false);
                    break;
                case HOTT_KEY_NEXT: // right
                    exit_menu    = false;
                    current_line = 0;
                    page = get_page(page, true);
                    break;
                case HOTT_KEY_SET: // Set
                    edit_line = true; // enter edit mode
                    break;
                }
            }

            // select page to display from text request
            // (this inhibit normal page browsing and give direct access to sensor page)
            switch (id_sensor) {
            case (HOTT_VARIO_ID & 0x0f):
                page = HOTTTEXT_PAGE_VARIOWARNINGS;
                break;
            case (HOTT_GPS_ID & 0x0f):
                page = HOTTTEXT_PAGE_GPS;
                break;
            case (HOTT_GAM_ID & 0x0f):
                page = HOTTTEXT_PAGE_GENERAL;
                break;
            case (HOTT_EAM_ID & 0x0f):
                page = HOTTTEXT_PAGE_ELECTRIC;
                break;
            case (HOTT_ESC_ID & 0x0f):
                page = HOTTTEXT_PAGE_ESC;
            }

            // new page
            if (page != last_page) {
                last_page    = page;
                current_line = 0;
                edit_line    = false;
            }

            // keep current line between min/max limits
            if (current_line > max_line[page]) {
                current_line = max_line[page];
            }
            if (current_line < min_line[page]) {
                current_line = min_line[page];
            }

            edit_status  = build_TEXT_message((struct hott_text_message *)tx_buffer, page, current_line, value_change, step_change, edit_line, exit_menu);
            message_size = sizeof(struct hott_text_message);
            if (edit_status == HOTTTEXT_EDITSTATUS_DONE) {
                // Save and exit edit mode
                store_settings(page, current_line);
                edit_line = false;
            } else if (edit_line) {
                step_change = edit_status;
            }
        }

        // check if a message is in the transmit buffer.
        if (message_size > 0) {
            status.RxPackets++;

            // check idle line before transmit. pause, then check receiver buffer
            vTaskDelayUntil(&lastSysTime, idledelay / portTICK_RATE_MS);

            if (PIOS_COM_ReceiveBuffer(PIOS_COM_HOTT, rx_buffer, 1, 0) == 0) {
                // nothing received means idle line. ready to transmit the requested message
                for (int i = 0; i < message_size; i++) {
                    // send message content with pause between each byte
                    PIOS_COM_SendCharNonBlocking(PIOS_COM_HOTT, tx_buffer[i]);
                    vTaskDelayUntil(&lastSysTime, datadelay / portTICK_RATE_MS);
                }
                status.TxPackets++;


                // after transmitting the message, any loopback data needs to be cleaned up.
                vTaskDelayUntil(&lastSysTime, idledelay / portTICK_RATE_MS);
                PIOS_COM_ReceiveBuffer(PIOS_COM_HOTT, tx_buffer, message_size, 0);
            } else {
                status.RxFail++;
            }
            HoTTBridgeStatusSet(&status);
        }
    }
}

/**
 * Build requested answer messages.
 * \return value sets message size
 */
uint16_t build_VARIO_message(struct hott_vario_message *msg)
{
    update_telemetrydata();

    if (telestate->Settings.Sensor.VARIO == HOTTBRIDGESETTINGS_SENSOR_DISABLED) {
        return 0;
    }

    // clear message buffer
    memset(msg, 0, sizeof(*msg));

    // message header
    msg->start     = HOTT_START;
    msg->stop      = HOTT_STOP;
    msg->sensor_id = HOTT_VARIO_ID;
    msg->warning   = generate_warning();
    msg->sensor_text_id = HOTT_VARIO_TEXT_ID;

    // alarm inverse bits. invert display areas on limits
    msg->alarm_inverse |= (telestate->Settings.Limit.MinHeight > telestate->altitude) ? VARIO_INVERT_ALT : 0;
    msg->alarm_inverse |= (telestate->Settings.Limit.MaxHeight < telestate->altitude) ? VARIO_INVERT_ALT : 0;
    msg->alarm_inverse |= (telestate->Settings.Limit.MaxHeight < telestate->altitude) ? VARIO_INVERT_MAX : 0;
    msg->alarm_inverse |= (telestate->Settings.Limit.MinHeight > telestate->altitude) ? VARIO_INVERT_MIN : 0;
    msg->alarm_inverse |= (telestate->Settings.Limit.NegDifference1 > telestate->climbrate1s) ? VARIO_INVERT_CR1S : 0;
    msg->alarm_inverse |= (telestate->Settings.Limit.PosDifference1 < telestate->climbrate1s) ? VARIO_INVERT_CR1S : 0;
    msg->alarm_inverse |= (telestate->Settings.Limit.NegDifference2 > telestate->climbrate3s) ? VARIO_INVERT_CR3S : 0;
    msg->alarm_inverse |= (telestate->Settings.Limit.PosDifference2 < telestate->climbrate3s) ? VARIO_INVERT_CR3S : 0;
    msg->alarm_inverse |= (telestate->Settings.Limit.NegDifference2 > telestate->climbrate10s) ? VARIO_INVERT_CR10S : 0;
    msg->alarm_inverse |= (telestate->Settings.Limit.PosDifference2 < telestate->climbrate10s) ? VARIO_INVERT_CR10S : 0;

    // altitude relative to ground
    msg->altitude     = scale_float2uword(telestate->altitude, 1, OFFSET_ALTITUDE);
    msg->min_altitude = scale_float2uword(telestate->min_altitude, 1, OFFSET_ALTITUDE);
    msg->max_altitude = scale_float2uword(telestate->max_altitude, 1, OFFSET_ALTITUDE);

    // climbrate
    msg->climbrate    = scale_float2uword(telestate->climbrate1s, M_TO_CM, OFFSET_CLIMBRATE);
    msg->climbrate3s  = scale_float2uword(telestate->climbrate3s, M_TO_CM, OFFSET_CLIMBRATE);
    msg->climbrate10s = scale_float2uword(telestate->climbrate10s, M_TO_CM, OFFSET_CLIMBRATE);

    // compass
    msg->compass = scale_float2int8(telestate->Attitude.Yaw, DEG_TO_UINT, 0);

    // statusline
    memcpy(msg->ascii, telestate->statusline, sizeof(msg->ascii));

    // free display characters
    msg->ascii1   = 0;
    msg->ascii2   = 0;
    msg->ascii3   = 0;

    msg->checksum = calc_checksum((uint8_t *)msg, sizeof(*msg));
    return sizeof(*msg);
}

uint16_t build_GPS_message(struct hott_gps_message *msg)
{
    update_telemetrydata();

    if (telestate->Settings.Sensor.GPS == HOTTBRIDGESETTINGS_SENSOR_DISABLED) {
        return 0;
    }

    // clear message buffer
    memset(msg, 0, sizeof(*msg));

    // message header
    msg->start     = HOTT_START;
    msg->stop      = HOTT_STOP;
    msg->sensor_id = HOTT_GPS_ID;
    msg->warning   = generate_warning();
    msg->sensor_text_id   = HOTT_GPS_TEXT_ID;

    // alarm inverse bits. invert display areas on limits
    msg->alarm_inverse1  |= (telestate->Settings.Limit.MaxDistance < telestate->homedistance) ? GPS_INVERT_HDIST : 0;
    msg->alarm_inverse1  |= (telestate->Settings.Limit.MinSpeed > telestate->GPS.Groundspeed) ? GPS_INVERT_SPEED : 0;
    msg->alarm_inverse1  |= (telestate->Settings.Limit.MaxSpeed < telestate->GPS.Groundspeed) ? GPS_INVERT_SPEED : 0;
    msg->alarm_inverse1  |= (telestate->Settings.Limit.MinHeight > telestate->altitude) ? GPS_INVERT_ALT : 0;
    msg->alarm_inverse1  |= (telestate->Settings.Limit.MaxHeight < telestate->altitude) ? GPS_INVERT_ALT : 0;
    msg->alarm_inverse1  |= (telestate->Settings.Limit.NegDifference1 > telestate->climbrate1s) ? GPS_INVERT_CR1S : 0;
    msg->alarm_inverse1  |= (telestate->Settings.Limit.PosDifference1 < telestate->climbrate1s) ? GPS_INVERT_CR1S : 0;
    msg->alarm_inverse1  |= (telestate->Settings.Limit.NegDifference2 > telestate->climbrate3s) ? GPS_INVERT_CR3S : 0;
    msg->alarm_inverse1  |= (telestate->Settings.Limit.PosDifference2 < telestate->climbrate3s) ? GPS_INVERT_CR3S : 0;
    msg->alarm_inverse2  |= (telestate->SysAlarms.Alarm.GPS != SYSTEMALARMS_ALARM_OK) ? GPS_INVERT2_POS : 0;

    // gps direction, groundspeed and position
    msg->flight_direction = scale_float2uint8(telestate->GPS.Heading, DEG_TO_UINT, 0);
    msg->gps_speed = scale_float2uword(get_redirect_sensor_value(HOTTBRIDGESETTINGS_SENSORREDIRECT_SPEED), MS_TO_KMH, 0);
    convert_long2gps(telestate->GPS.Latitude, &msg->latitude_ns, &msg->latitude_min, &msg->latitude_sec);
    convert_long2gps(telestate->GPS.Longitude, &msg->longitude_ew, &msg->longitude_min, &msg->longitude_sec);

    // homelocation distance, course and state
    msg->distance       = scale_float2uword(telestate->homedistance, 1, 0);
    msg->home_direction = scale_float2uint8(telestate->homecourse, DEG_TO_UINT, 0);
    msg->ascii5         = (telestate->Home.Set ? 'H' : '-');

    // altitude relative to ground and climb rate
    msg->altitude       = scale_float2uword(telestate->altitude, 1, OFFSET_ALTITUDE);
    msg->climbrate      = scale_float2uword(telestate->climbrate1s, M_TO_CM, OFFSET_CLIMBRATE);
    msg->climbrate3s    = scale_float2uint8(telestate->climbrate3s, 1, OFFSET_CLIMBRATE3S);

    // number of satellites,gps fix and state
    msg->gps_num_sat    = telestate->GPS.Satellites;
    switch (telestate->GPS.Status) {
    case GPSPOSITIONSENSOR_STATUS_FIX2D:
        msg->gps_fix_char = '2';
        break;
    case GPSPOSITIONSENSOR_STATUS_FIX3D:
    case GPSPOSITIONSENSOR_STATUS_FIX3DDGNSS:
        msg->gps_fix_char = '3';
        break;
    default:
        msg->gps_fix_char = 0;
    }
    switch (telestate->SysAlarms.Alarm.GPS) {
    case SYSTEMALARMS_ALARM_UNINITIALISED:
        msg->ascii6 = 0;
        // if there is no gps, show compass flight direction
        msg->flight_direction = scale_float2int8((telestate->Attitude.Yaw > 0) ? telestate->Attitude.Yaw : 360 + telestate->Attitude.Yaw, DEG_TO_UINT, 0);
        break;
    case SYSTEMALARMS_ALARM_OK:
        msg->ascii6 = '.';
        break;
    case SYSTEMALARMS_ALARM_WARNING:
        msg->ascii6 = '?';
        break;
    case SYSTEMALARMS_ALARM_ERROR:
    case SYSTEMALARMS_ALARM_CRITICAL:
        msg->ascii6 = '!';
        break;
    default:
        msg->ascii6 = 0;
    }

    // model angles
    msg->angle_roll    = scale_float2int8(telestate->Attitude.Roll, DEG_TO_UINT, 0);
    msg->angle_nick    = scale_float2int8(telestate->Attitude.Pitch, DEG_TO_UINT, 0);
    msg->angle_compass = scale_float2int8(telestate->Attitude.Yaw, DEG_TO_UINT, 0);

    // gps time
    msg->gps_hour = telestate->GPStime.Hour;
    msg->gps_min  = telestate->GPStime.Minute;
    msg->gps_sec  = telestate->GPStime.Second;
    msg->gps_msec = 0;

    // gps MSL (NN) altitude MSL
    msg->msl      = scale_float2uword(telestate->GPS.Altitude, 1, 0);

    // free display chararacter
    msg->ascii4   = 0;

    msg->checksum = calc_checksum((uint8_t *)msg, sizeof(*msg));
    return sizeof(*msg);
}

uint16_t build_GAM_message(struct hott_gam_message *msg)
{
    update_telemetrydata();

    if (telestate->Settings.Sensor.GAM == HOTTBRIDGESETTINGS_SENSOR_DISABLED) {
        return 0;
    }

    // clear message buffer
    memset(msg, 0, sizeof(*msg));

    // message header
    msg->start     = HOTT_START;
    msg->stop      = HOTT_STOP;
    msg->sensor_id = HOTT_GAM_ID;
    msg->warning   = generate_warning();
    msg->sensor_text_id  = HOTT_GAM_TEXT_ID;

    // alarm inverse bits. invert display areas on limits
    msg->alarm_inverse2 |= (telestate->Settings.Limit.MaxCurrent < telestate->Battery.Current) ? GAM_INVERT2_CURRENT : 0;
    msg->alarm_inverse2 |= (telestate->Settings.Limit.MinPowerVoltage > telestate->Battery.Voltage) ? GAM_INVERT2_VOLTAGE : 0;
    msg->alarm_inverse2 |= (telestate->Settings.Limit.MaxPowerVoltage < telestate->Battery.Voltage) ? GAM_INVERT2_VOLTAGE : 0;
    msg->alarm_inverse2 |= (telestate->Settings.Limit.MinHeight > telestate->altitude) ? GAM_INVERT2_ALT : 0;
    msg->alarm_inverse2 |= (telestate->Settings.Limit.MaxHeight < telestate->altitude) ? GAM_INVERT2_ALT : 0;
    msg->alarm_inverse2 |= (telestate->Settings.Limit.NegDifference1 > telestate->climbrate1s) ? GAM_INVERT2_CR1S : 0;
    msg->alarm_inverse2 |= (telestate->Settings.Limit.PosDifference1 < telestate->climbrate1s) ? GAM_INVERT2_CR1S : 0;
    msg->alarm_inverse2 |= (telestate->Settings.Limit.NegDifference2 > telestate->climbrate3s) ? GAM_INVERT2_CR3S : 0;
    msg->alarm_inverse2 |= (telestate->Settings.Limit.PosDifference2 < telestate->climbrate3s) ? GAM_INVERT2_CR3S : 0;

    // temperatures
    msg->temperature1    = scale_float2uint8(get_redirect_sensor_value(HOTTBRIDGESETTINGS_SENSORREDIRECT_TEMP1), 1, OFFSET_TEMPERATURE);
    msg->temperature2    = scale_float2uint8(get_redirect_sensor_value(HOTTBRIDGESETTINGS_SENSORREDIRECT_TEMP2), 1, OFFSET_TEMPERATURE);

    // altitude
    msg->altitude    = scale_float2uword(telestate->altitude, 1, OFFSET_ALTITUDE);

    // climbrate
    msg->climbrate   = scale_float2uword(telestate->climbrate1s, M_TO_CM, OFFSET_CLIMBRATE);
    msg->climbrate3s = scale_float2uint8(telestate->climbrate3s, 1, OFFSET_CLIMBRATE3S);

    // main battery
    float voltage = (telestate->Battery.Voltage > 0) ? telestate->Battery.Voltage : 0;
    float current = (telestate->Battery.Current > 0) ? telestate->Battery.Current : 0;
    float energy  = (telestate->Battery.ConsumedEnergy > 0) ? telestate->Battery.ConsumedEnergy : 0;
    msg->voltage  = scale_float2uword(voltage, 10, 0);
    msg->current  = scale_float2uword(current, 10, 0);
    msg->capacity = scale_float2uword(energy, 0.1f, 0);

    // simulate individual cell voltage
    uint8_t cell_voltage = (telestate->Battery.Voltage > 0) ? scale_float2uint8(telestate->Battery.Voltage / telestate->Battery.NbCells, 50, 0) : 0;
    msg->cell1 = (telestate->Battery.NbCells >= 1) ? cell_voltage : 0;
    msg->cell2 = (telestate->Battery.NbCells >= 2) ? cell_voltage : 0;
    msg->cell3 = (telestate->Battery.NbCells >= 3) ? cell_voltage : 0;
    msg->cell4 = (telestate->Battery.NbCells >= 4) ? cell_voltage : 0;
    msg->cell5 = (telestate->Battery.NbCells >= 5) ? cell_voltage : 0;
    msg->cell6 = (telestate->Battery.NbCells >= 6) ? cell_voltage : 0;

    msg->min_cell_volt     = (telestate->Battery.Voltage > 0) ? scale_float2uint8(telestate->min_voltage / telestate->Battery.NbCells, 50, 0) : 0;
    msg->min_cell_volt_num = telestate->Battery.NbCells;

    // batt1 and batt2 voltage
    msg->batt1_voltage     = scale_float2uword(get_redirect_sensor_value(HOTTBRIDGESETTINGS_SENSORREDIRECT_BATTERY1), 10, 0);
    msg->batt2_voltage     = scale_float2uword(get_redirect_sensor_value(HOTTBRIDGESETTINGS_SENSORREDIRECT_BATTERY2), 10, 0);

    // pressure kPa to 0.1Bar, max 25Bar
    msg->pressure = scale_float2uint8(get_redirect_sensor_value(HOTTBRIDGESETTINGS_SENSORREDIRECT_PRESSURE), 10, 0);

    msg->rpm = scale_float2uword(get_redirect_sensor_value(HOTTBRIDGESETTINGS_SENSORREDIRECT_RPM), 1, 0);

    msg->checksum = calc_checksum((uint8_t *)msg, sizeof(*msg));
    return sizeof(*msg);
}

uint16_t build_EAM_message(struct hott_eam_message *msg)
{
    update_telemetrydata();

    if (telestate->Settings.Sensor.EAM == HOTTBRIDGESETTINGS_SENSOR_DISABLED) {
        return 0;
    }

    // clear message buffer
    memset(msg, 0, sizeof(*msg));

    // message header
    msg->start     = HOTT_START;
    msg->stop      = HOTT_STOP;
    msg->sensor_id = HOTT_EAM_ID;
    msg->warning   = generate_warning();
    msg->sensor_text_id  = HOTT_EAM_TEXT_ID;

    // alarm inverse bits. invert display areas on limits
    msg->alarm_inverse1 |= (telestate->Settings.Limit.MaxUsedCapacity < telestate->Battery.ConsumedEnergy) ? EAM_INVERT_CAPACITY : 0;
    msg->alarm_inverse1 |= (telestate->Settings.Limit.MaxCurrent < telestate->Battery.Current) ? EAM_INVERT_CURRENT : 0;
    msg->alarm_inverse1 |= (telestate->Settings.Limit.MinPowerVoltage > telestate->Battery.Voltage) ? EAM_INVERT_VOLTAGE : 0;
    msg->alarm_inverse1 |= (telestate->Settings.Limit.MaxPowerVoltage < telestate->Battery.Voltage) ? EAM_INVERT_VOLTAGE : 0;
    msg->alarm_inverse2 |= (telestate->Settings.Limit.MinHeight > telestate->altitude) ? EAM_INVERT2_ALT : 0;
    msg->alarm_inverse2 |= (telestate->Settings.Limit.MaxHeight < telestate->altitude) ? EAM_INVERT2_ALT : 0;
    msg->alarm_inverse2 |= (telestate->Settings.Limit.NegDifference1 > telestate->climbrate1s) ? EAM_INVERT2_CR1S : 0;
    msg->alarm_inverse2 |= (telestate->Settings.Limit.PosDifference1 < telestate->climbrate1s) ? EAM_INVERT2_CR1S : 0;
    msg->alarm_inverse2 |= (telestate->Settings.Limit.NegDifference2 > telestate->climbrate3s) ? EAM_INVERT2_CR3S : 0;
    msg->alarm_inverse2 |= (telestate->Settings.Limit.PosDifference2 < telestate->climbrate3s) ? EAM_INVERT2_CR3S : 0;

    // main battery
    float voltage = (telestate->Battery.Voltage > 0) ? telestate->Battery.Voltage : 0;
    float current = (telestate->Battery.Current > 0) ? telestate->Battery.Current : 0;
    float energy  = (telestate->Battery.ConsumedEnergy > 0) ? telestate->Battery.ConsumedEnergy : 0;
    msg->voltage  = scale_float2uword(voltage, 10, 0);
    msg->current  = scale_float2uword(current, 10, 0);
    msg->capacity = scale_float2uword(energy, 0.1f, 0);

    // simulate individual cell voltage
    uint8_t cell_voltage = (telestate->Battery.Voltage > 0) ? scale_float2uint8(telestate->Battery.Voltage / telestate->Battery.NbCells, 50, 0) : 0;
    msg->cell1_H = (telestate->Battery.NbCells >= 1) ? cell_voltage : 0;
    msg->cell2_H = (telestate->Battery.NbCells >= 2) ? cell_voltage : 0;
    msg->cell3_H = (telestate->Battery.NbCells >= 3) ? cell_voltage : 0;
    msg->cell4_H = (telestate->Battery.NbCells >= 4) ? cell_voltage : 0;
    msg->cell5_H = (telestate->Battery.NbCells >= 5) ? cell_voltage : 0;
    msg->cell6_H = (telestate->Battery.NbCells >= 6) ? cell_voltage : 0;
    msg->cell7_H = (telestate->Battery.NbCells >= 7) ? cell_voltage : 0;

    uint8_t cell_voltage_min = (telestate->Battery.Voltage > 0) ? scale_float2uint8(telestate->min_voltage / telestate->Battery.NbCells, 50, 0) : 0;
    msg->cell1_L = (telestate->Battery.NbCells >= 1) ? cell_voltage_min : 0;
    msg->cell2_L = (telestate->Battery.NbCells >= 2) ? cell_voltage_min : 0;
    msg->cell3_L = (telestate->Battery.NbCells >= 3) ? cell_voltage_min : 0;
    msg->cell4_L = (telestate->Battery.NbCells >= 4) ? cell_voltage_min : 0;
    msg->cell5_L = (telestate->Battery.NbCells >= 5) ? cell_voltage_min : 0;
    msg->cell6_L = (telestate->Battery.NbCells >= 6) ? cell_voltage_min : 0;
    msg->cell7_L = (telestate->Battery.NbCells >= 7) ? cell_voltage_min : 0;

    // batt1 and batt2 voltage
    msg->batt1_voltage = scale_float2uword(get_redirect_sensor_value(HOTTBRIDGESETTINGS_SENSORREDIRECT_BATTERY1), 10, 0);
    msg->batt2_voltage = scale_float2uword(get_redirect_sensor_value(HOTTBRIDGESETTINGS_SENSORREDIRECT_BATTERY2), 10, 0);

    // temperatures
    msg->temperature1  = scale_float2uint8(get_redirect_sensor_value(HOTTBRIDGESETTINGS_SENSORREDIRECT_TEMP1), 1, OFFSET_TEMPERATURE);
    msg->temperature2  = scale_float2uint8(get_redirect_sensor_value(HOTTBRIDGESETTINGS_SENSORREDIRECT_TEMP2), 1, OFFSET_TEMPERATURE);

    // altitude
    msg->altitude    = scale_float2uword(telestate->altitude, 1, OFFSET_ALTITUDE);

    // climbrate
    msg->climbrate   = scale_float2uword(telestate->climbrate1s, M_TO_CM, OFFSET_CLIMBRATE);
    msg->climbrate3s = scale_float2uint8(telestate->climbrate3s, 1, OFFSET_CLIMBRATE3S);

    // flight time
    float flighttime = (telestate->Battery.EstimatedFlightTime <= 5999) ? telestate->Battery.EstimatedFlightTime : 5999;
    msg->electric_min = flighttime / 60;
    msg->electric_sec = flighttime - 60 * msg->electric_min;

    msg->rpm = scale_float2uword(get_redirect_sensor_value(HOTTBRIDGESETTINGS_SENSORREDIRECT_RPM), 1, 0);

    msg->checksum = calc_checksum((uint8_t *)msg, sizeof(*msg));
    return sizeof(*msg);
}

uint16_t build_ESC_message(struct hott_esc_message *msg)
{
    update_telemetrydata();

    if (telestate->Settings.Sensor.ESC == HOTTBRIDGESETTINGS_SENSOR_DISABLED) {
        return 0;
    }

    // clear message buffer
    memset(msg, 0, sizeof(*msg));

    // message header
    msg->start     = HOTT_START;
    msg->stop      = HOTT_STOP;
    msg->sensor_id = HOTT_ESC_ID;
    msg->warning   = 0;
    msg->sensor_text_id = HOTT_ESC_TEXT_ID;

    // main battery
    float voltage     = (telestate->Battery.Voltage > 0) ? telestate->Battery.Voltage : 0;
    float current     = (telestate->Battery.Current > 0) ? telestate->Battery.Current : 0;
    float max_current = (telestate->Battery.PeakCurrent > 0) ? telestate->Battery.PeakCurrent : 0;
    float energy = (telestate->Battery.ConsumedEnergy > 0) ? telestate->Battery.ConsumedEnergy : 0;
    msg->batt_voltage       = scale_float2uword(voltage, 10, 0);
    msg->current = scale_float2uword(current, 10, 0);
    msg->max_current        = scale_float2uword(max_current, 10, 0);
    msg->batt_capacity      = scale_float2uword(energy, 0.1f, 0);
    msg->min_batt_voltage   = scale_float2uword(telestate->min_voltage, 10, 0);

    // temperatures
    msg->temperatureESC     = scale_float2uint8(get_redirect_sensor_value(HOTTBRIDGESETTINGS_SENSORREDIRECT_TEMP1), 1, OFFSET_TEMPERATURE);
    msg->max_temperatureESC = scale_float2uint8(telestate->max_temp1, 1, OFFSET_TEMPERATURE);
    msg->temperatureMOT     = scale_float2uint8(get_redirect_sensor_value(HOTTBRIDGESETTINGS_SENSORREDIRECT_TEMP2), 1, OFFSET_TEMPERATURE);
    msg->max_temperatureMOT = scale_float2uint8(telestate->max_temp2, 1, OFFSET_TEMPERATURE);

    msg->rpm = scale_float2uword(get_redirect_sensor_value(HOTTBRIDGESETTINGS_SENSORREDIRECT_RPM), 1, 0);

    msg->checksum = calc_checksum((uint8_t *)msg, sizeof(*msg));
    return sizeof(*msg);
}

uint8_t build_TEXT_message(struct hott_text_message *msg, uint8_t page, uint8_t current_line, int8_t value_change, uint8_t step, bool edit_mode, bool exit_menu)
{
    update_telemetrydata();

    // clear message buffer
    memset(msg, 0, sizeof(*msg));

    // message header
    msg->start     = HOTT_TEXT_START;
    msg->stop      = HOTT_STOP;
    msg->sensor_id = (exit_menu == true) ? 0x01 : 0x00; // exit menu / normal
    msg->warning   = 0;

    HoTTBridgeSettingsSensorData sensor;
    HoTTBridgeSettingsLimitData alarmLimits;
    HoTTBridgeSettingsWarningData alarmWarning;
    uint8_t varioSensitivity;
    RevoSettingsFusionAlgorithmOptions revoFusionAlgo;
    AttitudeSettingsBoardRotationData boardRotation;
    FlightBatterySettingsSensorCalibrationsData battSensorCalibration;
    uint32_t battSensorCapacity;
    HomeLocationSetOptions homeSet;
    GPSSettingsData gpsSettings;
    uint8_t adcRouting[HWSETTINGS_ADCROUTING_NUMELEM];
    uint8_t sensorRedirect[HOTTBRIDGESETTINGS_SENSORREDIRECT_NUMELEM];

    uint8_t edit_status = step;

    // page title
    snprintf(msg->text[0], HOTT_TEXT_COLUMNS, "%s", hottTextPageTitle[page]); // line 1

    // compute page content
    switch (page) {
    case HOTTTEXT_PAGE_VARIOWARNINGS: // Vario page (Warnings)
        if (HoTTBridgeSettingsHandle() != NULL) {
            HoTTBridgeSettingsWarningGet(&alarmWarning);
        }

        bool edit_altitudebeep = (edit_mode && (current_line == 2));
        bool edit_maxheight    = (edit_mode && (current_line == 3));
        bool edit_minheight    = (edit_mode && (current_line == 4));
        bool edit_sinkrate1s   = (edit_mode && (current_line == 5));
        bool edit_climbrate1s  = (edit_mode && (current_line == 6));
        bool edit_sinkrate3s   = (edit_mode && (current_line == 7));
        bool edit_climbrate3s  = (edit_mode && (current_line == 8));

        if (edit_altitudebeep) {
            alarmWarning.AltitudeBeep = enable_disable_warning(alarmWarning.AltitudeBeep);
            HoTTBridgeSettingsWarningSet(&alarmWarning);
            edit_status = HOTTTEXT_EDITSTATUS_DONE;
        }
        if (edit_minheight) {
            alarmWarning.MinHeight = enable_disable_warning(alarmWarning.MinHeight);
            HoTTBridgeSettingsWarningSet(&alarmWarning);
            edit_status = HOTTTEXT_EDITSTATUS_DONE;
        }
        if (edit_maxheight) {
            alarmWarning.MaxHeight = enable_disable_warning(alarmWarning.MaxHeight);
            HoTTBridgeSettingsWarningSet(&alarmWarning);
            edit_status = HOTTTEXT_EDITSTATUS_DONE;
        }

        if (edit_sinkrate1s) {
            alarmWarning.NegDifference1 = enable_disable_warning(alarmWarning.NegDifference1);
            HoTTBridgeSettingsWarningSet(&alarmWarning);
            edit_status = HOTTTEXT_EDITSTATUS_DONE;
        }
        if (edit_climbrate1s) {
            alarmWarning.PosDifference1 = enable_disable_warning(alarmWarning.PosDifference1);
            HoTTBridgeSettingsWarningSet(&alarmWarning);
            edit_status = HOTTTEXT_EDITSTATUS_DONE;
        }
        if (edit_sinkrate3s) {
            alarmWarning.NegDifference2 = enable_disable_warning(alarmWarning.NegDifference2);
            HoTTBridgeSettingsWarningSet(&alarmWarning);
            edit_status = HOTTTEXT_EDITSTATUS_DONE;
        }
        if (edit_climbrate3s) {
            alarmWarning.PosDifference2 = enable_disable_warning(alarmWarning.PosDifference2);
            HoTTBridgeSettingsWarningSet(&alarmWarning);
            edit_status = HOTTTEXT_EDITSTATUS_DONE;
        }

        snprintf(msg->text[1], HOTT_TEXT_COLUMNS, " Altitude speak  [%1s]", ((alarmWarning.AltitudeBeep == HOTTBRIDGESETTINGS_WARNING_DISABLED) ? " " : "*")); // line 2
        snprintf(msg->text[2], HOTT_TEXT_COLUMNS, " MaxHeight       [%1s]", ((alarmWarning.MaxHeight == HOTTBRIDGESETTINGS_WARNING_DISABLED) ? " " : "*")); // line 3
        snprintf(msg->text[3], HOTT_TEXT_COLUMNS, " MinHeight       [%1s]", ((alarmWarning.MinHeight == HOTTBRIDGESETTINGS_WARNING_DISABLED) ? " " : "*")); // line 4
        snprintf(msg->text[4], HOTT_TEXT_COLUMNS, " Instant Sink    [%1s]", ((alarmWarning.NegDifference1 == HOTTBRIDGESETTINGS_WARNING_DISABLED) ? " " : "*")); // line 5
        snprintf(msg->text[5], HOTT_TEXT_COLUMNS, " Instant Climb   [%1s]", ((alarmWarning.PosDifference1 == HOTTBRIDGESETTINGS_WARNING_DISABLED) ? " " : "*")); // line 6
        snprintf(msg->text[6], HOTT_TEXT_COLUMNS, " Fast Sink       [%1s]", ((alarmWarning.NegDifference2 == HOTTBRIDGESETTINGS_WARNING_DISABLED) ? " " : "*")); // line 7
        snprintf(msg->text[7], HOTT_TEXT_COLUMNS, " Fast Climb      [%1s]", ((alarmWarning.PosDifference2 == HOTTBRIDGESETTINGS_WARNING_DISABLED) ? " " : "*")); // line 8
        if (current_line > 1) {
            msg->text[current_line - 1][0] = '>';
        }
        break;
    case HOTTTEXT_PAGE_VARIOLIMITS: // Vario page (Limits)
        if (HoTTBridgeSettingsHandle() != NULL) {
            HoTTBridgeSettingsLimitGet(&alarmLimits);
            HoTTBridgeSettingsVarioSensitivityGet(&varioSensitivity);
        }

        bool edit_sensitivity_value = (edit_mode && (current_line == 2));
        bool edit_maxheight_value   = (edit_mode && (current_line == 3));
        bool edit_minheight_value   = (edit_mode && (current_line == 4));
        bool edit_sinkrate1s_value  = (edit_mode && (current_line == 5));
        bool edit_climbrate1s_value = (edit_mode && (current_line == 6));
        bool edit_sinkrate3s_value  = (edit_mode && (current_line == 7));
        bool edit_climbrate3s_value = (edit_mode && (current_line == 8));

        if (edit_sensitivity_value) {
            step = (step > HOTTTEXT_EDITSTATUS_STEP10) ? HOTTTEXT_EDITSTATUS_STEP10 : step;
            // 0 to 99cm/s
            varioSensitivity = get_new_value((int16_t)varioSensitivity, value_change, step, 0, 99);
            HoTTBridgeSettingsVarioSensitivitySet(&varioSensitivity);
        }
        if (edit_minheight_value) {
            step = (step > HOTTTEXT_EDITSTATUS_STEP100) ? HOTTTEXT_EDITSTATUS_STEP100 : step;
            // -500 to 500m
            alarmLimits.MinHeight = get_new_value((int16_t)alarmLimits.MinHeight, value_change, step, -500, 500);
            HoTTBridgeSettingsLimitSet(&alarmLimits);
        }
        if (edit_maxheight_value) {
            step = (step > HOTTTEXT_EDITSTATUS_STEP100) ? HOTTTEXT_EDITSTATUS_STEP100 : step;
            // -500 to 1500m
            alarmLimits.MaxHeight = get_new_value((int16_t)alarmLimits.MaxHeight, value_change, step, -500, 1500);
            HoTTBridgeSettingsLimitSet(&alarmLimits);
        }
        if (edit_sinkrate1s_value) {
            step = (step > HOTTTEXT_EDITSTATUS_STEP10) ? HOTTTEXT_EDITSTATUS_STEP10 : step;
            // 0 to -50m
            alarmLimits.NegDifference1 = get_new_value((int16_t)alarmLimits.NegDifference1, value_change, step, -50, 0);
            HoTTBridgeSettingsLimitSet(&alarmLimits);
        }
        if (edit_climbrate1s_value) {
            step = (step > HOTTTEXT_EDITSTATUS_STEP10) ? HOTTTEXT_EDITSTATUS_STEP10 : step;
            // 0 to 50m
            alarmLimits.PosDifference1 = get_new_value((int16_t)alarmLimits.PosDifference1, value_change, step, 0, 50);
            HoTTBridgeSettingsLimitSet(&alarmLimits);
        }
        if (edit_sinkrate3s_value) {
            step = (step > HOTTTEXT_EDITSTATUS_STEP100) ? HOTTTEXT_EDITSTATUS_STEP100 : step;
            // 0 to -500m
            alarmLimits.NegDifference2 = get_new_value((int16_t)alarmLimits.NegDifference2, value_change, step, -500, 0);
            HoTTBridgeSettingsLimitSet(&alarmLimits);
        }
        if (edit_climbrate3s_value) {
            step = (step > HOTTTEXT_EDITSTATUS_STEP100) ? HOTTTEXT_EDITSTATUS_STEP100 : step;
            // 0 to 500m
            alarmLimits.PosDifference2 = get_new_value((int16_t)alarmLimits.PosDifference2, value_change, step, 0, 500);
            HoTTBridgeSettingsLimitSet(&alarmLimits);
        }

        snprintf(msg->text[1], HOTT_TEXT_COLUMNS, " Sensitivity cm/s%3d", (int16_t)varioSensitivity); // line 2
        snprintf(msg->text[2], HOTT_TEXT_COLUMNS, " Max height   % 6d", (int16_t)alarmLimits.MaxHeight); // line 3
        snprintf(msg->text[3], HOTT_TEXT_COLUMNS, " Min height   % 6d", (int16_t)alarmLimits.MinHeight); // line 4
        snprintf(msg->text[4], HOTT_TEXT_COLUMNS, " Inst. Sink m/s % 3d", maxd3((int16_t)alarmLimits.NegDifference1)); // line 5
        snprintf(msg->text[5], HOTT_TEXT_COLUMNS, " Inst. Clmb m/s % 3d", maxd3((int16_t)alarmLimits.PosDifference1)); // line 6
        snprintf(msg->text[6], HOTT_TEXT_COLUMNS, " Fast Sink m/3s% 4d", maxd4((int16_t)alarmLimits.NegDifference2)); // line 7
        snprintf(msg->text[7], HOTT_TEXT_COLUMNS, " Fast Clmb m/3s% 4d", maxd4((int16_t)alarmLimits.PosDifference2)); // line 8
        if (current_line > 1) {
            msg->text[current_line - 1][0] = '>';
        }

        if (edit_minheight_value || edit_maxheight_value) {
            reverse_pixels((char *)msg->text[current_line - 1], 16 + (3 - step), 20 - step);
        }
        if (edit_sensitivity_value || edit_sinkrate1s_value || edit_climbrate1s_value) {
            reverse_pixels((char *)msg->text[current_line - 1], 18 + (1 - step), 20 - step);
        }
        if (edit_sinkrate3s_value || edit_climbrate3s_value) {
            reverse_pixels((char *)msg->text[current_line - 1], 17 + (2 - step), 20 - step);
        }
        break;
    case HOTTTEXT_PAGE_GPS: // GPS page
        if (HoTTBridgeSettingsHandle() != NULL) {
            HoTTBridgeSettingsWarningGet(&alarmWarning);
            HoTTBridgeSettingsLimitGet(&alarmLimits);
        }

        bool edit_maxdistance       = (edit_mode && (current_line == 2));
        bool edit_maxspeed          = (edit_mode && (current_line == 3));
        bool edit_minspeed          = (edit_mode && (current_line == 4));
        bool edit_maxdistance_value = (edit_mode && (current_line == 5));
        bool edit_maxspeed_value    = (edit_mode && (current_line == 6));
        bool edit_minspeed_value    = (edit_mode && (current_line == 7));

        if (edit_maxdistance) {
            alarmWarning.MaxDistance = enable_disable_warning(alarmWarning.MaxDistance);
            HoTTBridgeSettingsWarningSet(&alarmWarning);
            edit_status = HOTTTEXT_EDITSTATUS_DONE;
        }
        if (edit_minspeed) {
            alarmWarning.MinSpeed = enable_disable_warning(alarmWarning.MinSpeed);
            HoTTBridgeSettingsWarningSet(&alarmWarning);
            edit_status = HOTTTEXT_EDITSTATUS_DONE;
        }
        if (edit_maxspeed) {
            alarmWarning.MaxSpeed = enable_disable_warning(alarmWarning.MaxSpeed);
            HoTTBridgeSettingsWarningSet(&alarmWarning);
            edit_status = HOTTTEXT_EDITSTATUS_DONE;
        }

        if (edit_maxdistance_value) {
            step = (step > HOTTTEXT_EDITSTATUS_STEP1K) ? HOTTTEXT_EDITSTATUS_STEP1K : step;
            // 10m to 9000m
            alarmLimits.MaxDistance = get_new_value((uint16_t)alarmLimits.MaxDistance, value_change, step, 10, 9000);
            HoTTBridgeSettingsLimitSet(&alarmLimits);
        }
        if (edit_maxspeed_value) {
            // 0kmh to 1000kmh
            step = (step > HOTTTEXT_EDITSTATUS_STEP100) ? HOTTTEXT_EDITSTATUS_STEP100 : step;
            alarmLimits.MaxSpeed = get_new_value((int16_t)alarmLimits.MaxSpeed, value_change, step, 0, 1000);
            HoTTBridgeSettingsLimitSet(&alarmLimits);
        }
        if (edit_minspeed_value) {
            // 0kmh to 1000kmh
            step = (step > HOTTTEXT_EDITSTATUS_STEP100) ? HOTTTEXT_EDITSTATUS_STEP100 : step;
            alarmLimits.MinSpeed = get_new_value((int16_t)alarmLimits.MinSpeed, value_change, step, 0, 1000);
            HoTTBridgeSettingsLimitSet(&alarmLimits);
        }
        snprintf(msg->text[1], HOTT_TEXT_COLUMNS, " MaxDist warn    [%1s]", ((alarmWarning.MaxDistance == HOTTBRIDGESETTINGS_WARNING_DISABLED) ? " " : "*")); // line 2
        snprintf(msg->text[2], HOTT_TEXT_COLUMNS, " MaxSpeed warn   [%1s]", ((alarmWarning.MaxSpeed == HOTTBRIDGESETTINGS_WARNING_DISABLED) ? " " : "*")); // line 3
        snprintf(msg->text[3], HOTT_TEXT_COLUMNS, " MinSpeed warn   [%1s]", ((alarmWarning.MinSpeed == HOTTBRIDGESETTINGS_WARNING_DISABLED) ? " " : "*")); // line 4
        snprintf(msg->text[4], HOTT_TEXT_COLUMNS, " Max distance  %5d", (uint16_t)alarmLimits.MaxDistance); // line 5
        snprintf(msg->text[5], HOTT_TEXT_COLUMNS, " Max speed    % 5d", (int16_t)alarmLimits.MaxSpeed); // line 6
        snprintf(msg->text[6], HOTT_TEXT_COLUMNS, " Min speed    % 5d", (int16_t)alarmLimits.MinSpeed); // line 7
        snprintf(msg->text[7], HOTT_TEXT_COLUMNS, "                    "); // line 8
        if (current_line > 1) {
            msg->text[current_line - 1][0] = '>';
        }

        if (edit_maxdistance_value) {
            reverse_pixels((char *)msg->text[current_line - 1], 16 + (3 - step), 20 - step);
        }
        if (edit_maxspeed_value) {
            reverse_pixels((char *)msg->text[current_line - 1], 16 + (3 - step), 20 - step);
        }
        if (edit_minspeed_value) {
            reverse_pixels((char *)msg->text[current_line - 1], 16 + (3 - step), 20 - step);
        }
        break;
    case HOTTTEXT_PAGE_GENERAL: // General Air page
    case HOTTTEXT_PAGE_ELECTRIC: // Electric Air page
    case HOTTTEXT_PAGE_ESC: // Esc page
        if (HoTTBridgeSettingsHandle() != NULL) {
            HoTTBridgeSettingsWarningGet(&alarmWarning);
            HoTTBridgeSettingsLimitGet(&alarmLimits);
        }
        if (FlightBatterySettingsHandle() != NULL) {
            FlightBatterySettingsCapacityGet(&battSensorCapacity);
        }

        bool edit_minvoltage = (edit_mode && (current_line == 2));
        bool edit_maxcurrent = (edit_mode && (current_line == 3));
        bool edit_maxusedcapacity  = (edit_mode && (current_line == 4));
        bool edit_minvoltage_value = (edit_mode && (current_line == 5));
        bool edit_maxcurrent_value = (edit_mode && (current_line == 6));
        bool edit_maxusedcapacity_value = (edit_mode && (current_line == 7));

        if (edit_minvoltage) {
            alarmWarning.MinPowerVoltage = enable_disable_warning(alarmWarning.MinPowerVoltage);
            HoTTBridgeSettingsWarningSet(&alarmWarning);
            edit_status = HOTTTEXT_EDITSTATUS_DONE;
        }
        if (edit_maxcurrent) {
            alarmWarning.MaxCurrent = enable_disable_warning(alarmWarning.MaxCurrent);
            HoTTBridgeSettingsWarningSet(&alarmWarning);
            edit_status = HOTTTEXT_EDITSTATUS_DONE;
        }
        if (edit_maxusedcapacity) {
            alarmWarning.MaxUsedCapacity = enable_disable_warning(alarmWarning.MaxUsedCapacity);
            HoTTBridgeSettingsWarningSet(&alarmWarning);
            edit_status = HOTTTEXT_EDITSTATUS_DONE;
        }

        if (edit_minvoltage_value) {
            step = (step > HOTTTEXT_EDITSTATUS_STEP100) ? HOTTTEXT_EDITSTATUS_STEP100 : step;
            // 3V to 50V
            alarmLimits.MinPowerVoltage = (float)(get_new_value((uint16_t)(alarmLimits.MinPowerVoltage * 10), value_change, step, 30, 500) / 10.0f);
            HoTTBridgeSettingsLimitSet(&alarmLimits);
        }
        if (edit_maxcurrent_value) {
            step = (step > HOTTTEXT_EDITSTATUS_STEP100) ? HOTTTEXT_EDITSTATUS_STEP100 : step;
            // 1A to 300A
            alarmLimits.MaxCurrent = get_new_value((uint16_t)(alarmLimits.MaxCurrent), value_change, step, 1, 300);
            HoTTBridgeSettingsLimitSet(&alarmLimits);
        }
        if (edit_maxusedcapacity_value) {
            step = (step > HOTTTEXT_EDITSTATUS_STEP1K) ? HOTTTEXT_EDITSTATUS_STEP1K : step;
            // 100mAh to 30000mAh
            alarmLimits.MaxUsedCapacity = (float)(get_new_value((uint16_t)alarmLimits.MaxUsedCapacity, value_change, step, 100, 30000));
            HoTTBridgeSettingsLimitSet(&alarmLimits);
            // apply MaxUsedCapacity as main battery capacity
            battSensorCapacity = (uint32_t)alarmLimits.MaxUsedCapacity;
            FlightBatterySettingsCapacitySet(&battSensorCapacity);
        }

        snprintf(msg->text[1], HOTT_TEXT_COLUMNS, " MinVoltage warn [%1s]", ((alarmWarning.MinPowerVoltage == HOTTBRIDGESETTINGS_WARNING_DISABLED) ? " " : "*")); // line 2
        snprintf(msg->text[2], HOTT_TEXT_COLUMNS, " MaxCurrent warn [%1s]", ((alarmWarning.MaxCurrent == HOTTBRIDGESETTINGS_WARNING_DISABLED) ? " " : "*")); // line 3
        snprintf(msg->text[3], HOTT_TEXT_COLUMNS, " MaxUsedmAH warn [%1s]", ((alarmWarning.MaxUsedCapacity == HOTTBRIDGESETTINGS_WARNING_DISABLED) ? " " : "*")); // line 4
        snprintf(msg->text[4], HOTT_TEXT_COLUMNS, " Min voltage    %2d.%d", maxu2((uint16_t)(alarmLimits.MinPowerVoltage)), (uint16_t)(alarmLimits.MinPowerVoltage * 10) % 10); // line 5
        snprintf(msg->text[5], HOTT_TEXT_COLUMNS, " Max current    %4d", maxu4((uint16_t)alarmLimits.MaxCurrent)); // line 6
        snprintf(msg->text[6], HOTT_TEXT_COLUMNS, " Max used mAH  %5d", (uint16_t)alarmLimits.MaxUsedCapacity); // line 7
        snprintf(msg->text[7], HOTT_TEXT_COLUMNS, "                    "); // line 8
        if (current_line > 1) {
            msg->text[current_line - 1][0] = '>';
        }

        if (edit_minvoltage_value) {
            reverse_pixels((char *)msg->text[current_line - 1], 15 + (4 - (step > 0 ? step + 1 : step)), 20 - (step > 0 ? step + 1 : step));
        }
        if (edit_maxcurrent_value) {
            reverse_pixels((char *)msg->text[current_line - 1], 16 + (3 - step), 20 - step);
        }
        if (edit_maxusedcapacity_value) {
            reverse_pixels((char *)msg->text[current_line - 1], 15 + (4 - step), 20 - step);
        }
        break;
    case HOTTTEXT_PAGE_SENSORREDIR:
        if (HoTTBridgeSettingsHandle() != NULL) {
            HoTTBridgeSettingsSensorRedirectArrayGet(sensorRedirect);
        }

        if (edit_mode) {
            uint8_t sensor_data = current_line - 2;
            if ((value_change > 0) && (sensorRedirect[sensor_data] < HOTTBRIDGESETTINGS_SENSORREDIRECT_GFORCE)) {
                sensorRedirect[sensor_data]++;
            } else if ((value_change < 0) && (sensorRedirect[sensor_data] > HOTTBRIDGESETTINGS_SENSORREDIRECT_NONE)) {
                sensorRedirect[sensor_data]--;
            }
            HoTTBridgeSettingsSensorRedirectArraySet(sensorRedirect);
        }

        snprintf(msg->text[1], HOTT_TEXT_COLUMNS, " Speed      %s ", hottTextSensorRedirectNames[sensorRedirect[HOTTBRIDGESETTINGS_SENSORREDIRECT_SPEED]]); // line 2
        snprintf(msg->text[2], HOTT_TEXT_COLUMNS, " Battery1   %s ", hottTextSensorRedirectNames[sensorRedirect[HOTTBRIDGESETTINGS_SENSORREDIRECT_BATTERY1]]); // line 3
        snprintf(msg->text[3], HOTT_TEXT_COLUMNS, " Battery2   %s ", hottTextSensorRedirectNames[sensorRedirect[HOTTBRIDGESETTINGS_SENSORREDIRECT_BATTERY2]]); // line 4
        snprintf(msg->text[4], HOTT_TEXT_COLUMNS, " Temp1      %s ", hottTextSensorRedirectNames[sensorRedirect[HOTTBRIDGESETTINGS_SENSORREDIRECT_TEMP1]]); // line 5
        snprintf(msg->text[5], HOTT_TEXT_COLUMNS, " Temp2      %s ", hottTextSensorRedirectNames[sensorRedirect[HOTTBRIDGESETTINGS_SENSORREDIRECT_TEMP2]]); // line 6
        snprintf(msg->text[6], HOTT_TEXT_COLUMNS, " Pressure   %s ", hottTextSensorRedirectNames[sensorRedirect[HOTTBRIDGESETTINGS_SENSORREDIRECT_PRESSURE]]); // line 7
        snprintf(msg->text[7], HOTT_TEXT_COLUMNS, " RPM        %s ", hottTextSensorRedirectNames[sensorRedirect[HOTTBRIDGESETTINGS_SENSORREDIRECT_RPM]]); // line 8
        if (current_line > 1) {
            msg->text[current_line - 1][0] = '>';
        }
        if (edit_mode) {
            reverse_pixels((char *)msg->text[current_line - 1], 12, 20);
        }
        break;
    case HOTTTEXT_PAGE_GPSCONFIG: // GPS config page
        if (GPSSettingsHandle() != NULL) {
            GPSSettingsGet(&gpsSettings);
        }
        if (HomeLocationHandle() != NULL) {
            HomeLocationSetGet(&homeSet);
        }

        bool edit_savehome      = (edit_mode && (current_line == 2));
        bool edit_minsat_value  = (edit_mode && (current_line == 3));
        bool edit_maxpdop_value = (edit_mode && (current_line == 4));
        bool edit_ubxrate_value = (edit_mode && (current_line == 5));

        if (edit_minsat_value) {
            gpsSettings.MinSatellites = get_new_value(gpsSettings.MinSatellites, value_change, 0, 4, 9);
            GPSSettingsSet(&gpsSettings);
        }
        if (edit_maxpdop_value) {
            step = (step > HOTTTEXT_EDITSTATUS_STEP100) ? HOTTTEXT_EDITSTATUS_STEP100 : step;
            // 1.0 to 10.0
            gpsSettings.MaxPDOP = (float)(get_new_value((uint16_t)(gpsSettings.MaxPDOP * 10), value_change, step, 10, 100) / 10.0f);
            GPSSettingsSet(&gpsSettings);
        }
        if (edit_ubxrate_value) {
            step = (step > HOTTTEXT_EDITSTATUS_STEP1) ? HOTTTEXT_EDITSTATUS_STEP1 : step;
            gpsSettings.UbxRate = get_new_value(gpsSettings.UbxRate, value_change, 0, 1, 15);
            GPSSettingsSet(&gpsSettings);
        }

        char *home_set_status = (homeSetFlash == HOMELOCATION_SET_FALSE) ? ((homeSet == HOMELOCATION_SET_TRUE) ? "ISSET" : "    ?") : "FIXED";

        snprintf(msg->text[1], HOTT_TEXT_COLUMNS, " Home status   %s", home_set_status); // line 2
        snprintf(msg->text[2], HOTT_TEXT_COLUMNS, " Min satellites  %u", (uint8_t)gpsSettings.MinSatellites); // line 3
        snprintf(msg->text[3], HOTT_TEXT_COLUMNS, " Max PDOP       %2d.%d", maxu2((uint16_t)(gpsSettings.MaxPDOP)), (uint16_t)(gpsSettings.MaxPDOP * 10) % 10); // line 4
        snprintf(msg->text[4], HOTT_TEXT_COLUMNS, " UBX Rate        % 2d", maxd2(gpsSettings.UbxRate)); // line 5
        snprintf(msg->text[5], HOTT_TEXT_COLUMNS, "                    "); // line 6
        snprintf(msg->text[6], HOTT_TEXT_COLUMNS, "                    "); // line 7
        snprintf(msg->text[7], HOTT_TEXT_COLUMNS, "%2d Sats %s PDOP:%2d.%d",
                 maxu2((uint16_t)(telestate->GPS.Satellites)),
                 ((telestate->GPS.Status > GPSPOSITIONSENSOR_STATUS_FIX2D) ? "3D" : "??"),
                 maxu2((uint16_t)(telestate->GPS.PDOP)), (uint16_t)(telestate->GPS.PDOP * 10) % 10); // line 8
        if (current_line > 1) {
            msg->text[current_line - 1][0] = '>';
        }

        if (edit_savehome) {
            if (homeSet == HOMELOCATION_SET_TRUE) {
                homeSet = HOMELOCATION_SET_FALSE;
                HomeLocationSetSet(&homeSet);
            }
            // refresh fixed homelocation if any
            if (homeSetFlash == HOMELOCATION_SET_TRUE) {
                homeSetFlash = HOMELOCATION_SET_FALSE;
                HomeLocationSetSet(&homeSetFlash);
                UAVObjSave(HomeLocationHandle(), 0);
            }
            edit_status = HOTTTEXT_EDITSTATUS_DONE;
        }
        if (edit_minsat_value) {
            reverse_pixels((char *)msg->text[current_line - 1], 19, 20);
        }
        if (edit_maxpdop_value) {
            reverse_pixels((char *)msg->text[current_line - 1], 15 + (4 - (step > 0 ? step + 1 : step)), 20 - (step > 0 ? step + 1 : step));
        }
        if (edit_ubxrate_value) {
            reverse_pixels((char *)msg->text[current_line - 1], 18, 20);
        }
        break;
    case HOTTTEXT_PAGE_BATTERYCONFIG:
        if (FlightBatterySettingsHandle() != NULL) {
            FlightBatterySettingsSensorCalibrationsGet(&battSensorCalibration);
        }
        if (HwSettingsHandle() != NULL) {
            HwSettingsADCRoutingArrayGet(adcRouting);
        }
        if (edit_mode && (current_line == 2)) {
            step = (step > HOTTTEXT_EDITSTATUS_STEP1K) ? HOTTTEXT_EDITSTATUS_STEP1K : step;
            // 1.00 to 30.00
            battSensorCalibration.VoltageFactor = (float)(get_new_value((uint16_t)roundf(battSensorCalibration.VoltageFactor * 100), value_change, step, 100, 3000) / 100.0f);
            FlightBatterySettingsSensorCalibrationsSet(&battSensorCalibration);
        }
        if (edit_mode && (current_line == 3)) {
            step = (step > HOTTTEXT_EDITSTATUS_STEP1K) ? HOTTTEXT_EDITSTATUS_STEP1K : step;
            // 1.00 to 30.00
            battSensorCalibration.CurrentFactor = (float)(get_new_value((uint16_t)roundf(battSensorCalibration.CurrentFactor * 100), value_change, step, 100, 3000) / 100.0f);
            FlightBatterySettingsSensorCalibrationsSet(&battSensorCalibration);
        }
        if (edit_mode && (current_line == 4)) {
            step = (step > HOTTTEXT_EDITSTATUS_STEP100) ? HOTTTEXT_EDITSTATUS_STEP100 : step;
            // -9.00 to 9.00
            battSensorCalibration.CurrentZero = (float)(get_new_value((int16_t)roundf(battSensorCalibration.CurrentZero * 100), value_change, step, -900, 900) / 100.0f);
            FlightBatterySettingsSensorCalibrationsSet(&battSensorCalibration);
        }

        int8_t voltageADCPin = ADC_XX_PIN_NOTFOUND;
        int8_t currentADCPin = ADC_XX_PIN_NOTFOUND;
        int8_t newADCPin     = ADC_XX_PIN_NOTFOUND;

        for (int i = 0; i < HWSETTINGS_ADCROUTING_NUMELEM; i++) {
            if (adcRouting[i] == HWSETTINGS_ADCROUTING_BATTERYVOLTAGE) {
                voltageADCPin = i;
            }
            if (adcRouting[i] == HWSETTINGS_ADCROUTING_BATTERYCURRENT) {
                currentADCPin = i;
            }
        }

        if (edit_mode && (current_line == 5)) {
            newADCPin = get_newADCPin_value(adcRouting, voltageADCPin, value_change);
            adcRouting[voltageADCPin] = HWSETTINGS_ADCROUTING_DISABLED;
            if (newADCPin > ADC_XX_PIN_NOTFOUND) {
                adcRouting[newADCPin] = HWSETTINGS_ADCROUTING_BATTERYVOLTAGE;
            }
            HwSettingsADCRoutingArraySet(adcRouting);
        }
        if (edit_mode && (current_line == 6)) {
            newADCPin = get_newADCPin_value(adcRouting, currentADCPin, value_change);
            adcRouting[currentADCPin] = HWSETTINGS_ADCROUTING_DISABLED;
            if (newADCPin > ADC_XX_PIN_NOTFOUND) {
                adcRouting[newADCPin] = HWSETTINGS_ADCROUTING_BATTERYCURRENT;
            }
            HwSettingsADCRoutingArraySet(adcRouting);
        }
        snprintf(msg->text[1], HOTT_TEXT_COLUMNS, " Volt. Factor  %2d.%02d", maxu2((uint16_t)(battSensorCalibration.VoltageFactor)), (uint16_t)(battSensorCalibration.VoltageFactor * 100) % 100); // line 2
        snprintf(msg->text[2], HOTT_TEXT_COLUMNS, " Curr. Factor  %2d.%02d", maxu2((uint16_t)(battSensorCalibration.CurrentFactor)), (uint16_t)(battSensorCalibration.CurrentFactor * 100) % 100); // line 3
        snprintf(msg->text[3], HOTT_TEXT_COLUMNS, " Curr. Zero %s%d.%02d", (battSensorCalibration.CurrentZero < 0 ? "-" : " "), maxu4((uint16_t)(fabsf(battSensorCalibration.CurrentZero))), (uint16_t)(fabsf(battSensorCalibration.CurrentZero * 100)) % 100); // line 4
        snprintf(msg->text[4], HOTT_TEXT_COLUMNS, " VoltagePin     %s", hottTextADCpinNames[voltageADCPin + 1]); // line 5
        snprintf(msg->text[5], HOTT_TEXT_COLUMNS, " CurrentPin     %s", hottTextADCpinNames[currentADCPin + 1]); // line 6
        snprintf(msg->text[6], HOTT_TEXT_COLUMNS, "                    "); // line 7
        snprintf(msg->text[7], HOTT_TEXT_COLUMNS, "%2d.%02dV %3d.%02dA     ",
                 maxu2((uint16_t)(telestate->Battery.Voltage)), (uint16_t)(telestate->Battery.Voltage * 100) % 100,
                 maxu3((uint16_t)(telestate->Battery.Current)), (uint16_t)(telestate->Battery.Current * 100) % 100); // line 8
        if (current_line > 1) {
            msg->text[current_line - 1][0] = '>';
        }
        if (edit_mode && ((current_line == 2) || (current_line == 3) || (current_line == 4))) {
            reverse_pixels((char *)msg->text[current_line - 1], 14 + (5 - (step > 1 ? step + 1 : step)), 20 - (step > 1 ? step + 1 : step));
        }
        if (edit_mode && ((current_line == 5) || (current_line == 6))) {
            reverse_pixels((char *)msg->text[current_line - 1], 16, 20);
        }
        break;
    case HOTTTEXT_PAGE_MAINCONFIG:
        if (RevoSettingsHandle() != NULL) {
            RevoSettingsFusionAlgorithmGet(&revoFusionAlgo);
        }
        if (AttitudeSettingsHandle() != NULL) {
            AttitudeSettingsBoardRotationGet(&boardRotation);
        }
        if (edit_mode && (current_line == 2)) {
            if ((value_change > 0) && (revoFusionAlgo < REVOSETTINGS_FUSIONALGORITHM_ACRONOSENSORS)) {
                revoFusionAlgo++;
            } else if ((value_change < 0) && (revoFusionAlgo > REVOSETTINGS_FUSIONALGORITHM_NONE)) {
                revoFusionAlgo--;
            }
            RevoSettingsFusionAlgorithmSet(&revoFusionAlgo);
        }
        char *txt_fusionalgo = "";
        // check current algo status
        switch (revoFusionAlgo) {
        case REVOSETTINGS_FUSIONALGORITHM_INS13INDOOR:
            txt_fusionalgo = " INS NOGPS ";
            break;
        case REVOSETTINGS_FUSIONALGORITHM_TESTINGINSINDOORCF:
            txt_fusionalgo = "INS+CFNOGPS";
            break;
        case REVOSETTINGS_FUSIONALGORITHM_GPSNAVIGATIONINS13:
            txt_fusionalgo = "  INS GPS  ";
            break;
        case REVOSETTINGS_FUSIONALGORITHM_GPSNAVIGATIONINS13CF:
            txt_fusionalgo = " INS+CF GPS";
            break;
        case REVOSETTINGS_FUSIONALGORITHM_BASICCOMPLEMENTARY:
            txt_fusionalgo = "   BASIC   ";
            break;
        case REVOSETTINGS_FUSIONALGORITHM_COMPLEMENTARYGPSOUTDOOR:
            txt_fusionalgo = " BASIC+GPS ";
            break;
        case REVOSETTINGS_FUSIONALGORITHM_COMPLEMENTARYMAG:
            txt_fusionalgo = " BASIC+MAG ";
            break;
        case REVOSETTINGS_FUSIONALGORITHM_COMPLEMENTARYMAGGPSOUTDOOR:
            txt_fusionalgo = "BASICMAGGPS";
            break;
        case REVOSETTINGS_FUSIONALGORITHM_ACRONOSENSORS:
            txt_fusionalgo = " ??ACRO??  ";
            break;
        case REVOSETTINGS_FUSIONALGORITHM_NONE:
            txt_fusionalgo = " ??NONE??  ";
            break;
        default:
            txt_fusionalgo = "!UNDEFINED!";
        }
        if (edit_mode && (current_line == 3)) {
            step = (step > HOTTTEXT_EDITSTATUS_STEP100) ? HOTTTEXT_EDITSTATUS_STEP1K : step;
            // -180 to 180
            boardRotation.Roll = (float)(get_new_value((int16_t)(boardRotation.Roll * 10), value_change, step, -1800, 1800) / 10.0f);
            AttitudeSettingsBoardRotationSet(&boardRotation);
        }
        if (edit_mode && (current_line == 4)) {
            step = (step > HOTTTEXT_EDITSTATUS_STEP100) ? HOTTTEXT_EDITSTATUS_STEP1K : step;
            // -180 to 180
            boardRotation.Pitch = (float)(get_new_value((int16_t)(boardRotation.Pitch * 10), value_change, step, -1800, 1800) / 10.0f);
            AttitudeSettingsBoardRotationSet(&boardRotation);
        }
        if (edit_mode && (current_line == 5)) {
            step = (step > HOTTTEXT_EDITSTATUS_STEP100) ? HOTTTEXT_EDITSTATUS_STEP1K : step;
            // -180 to 180
            boardRotation.Yaw = (float)(get_new_value((int16_t)(boardRotation.Yaw * 10), value_change, step, -1800, 1800) / 10.0f);
            AttitudeSettingsBoardRotationSet(&boardRotation);
        }

        snprintf(msg->text[1], HOTT_TEXT_COLUMNS, " EstAlgo %s", txt_fusionalgo); // line 2
        snprintf(msg->text[2], HOTT_TEXT_COLUMNS, " Roll rot.   % 4d.%d", maxd4((int16_t)(boardRotation.Roll)), (int16_t)fabs(boardRotation.Roll * 10) % 10); // line 3
        snprintf(msg->text[3], HOTT_TEXT_COLUMNS, " Pitch rot.  % 4d.%d", maxd4((int16_t)(boardRotation.Pitch)), (int16_t)fabs(boardRotation.Pitch * 10) % 10); // line 4
        snprintf(msg->text[4], HOTT_TEXT_COLUMNS, " Yaw rot.    % 4d.%d", maxd4((int16_t)(boardRotation.Yaw)), (int16_t)fabs(boardRotation.Yaw * 10) % 10); // line 5
        snprintf(msg->text[5], HOTT_TEXT_COLUMNS, "                    "); // line 6
        snprintf(msg->text[6], HOTT_TEXT_COLUMNS, "                    "); // line 7
        snprintf(msg->text[7], HOTT_TEXT_COLUMNS, "                    "); // line 8
        if (current_line > 1) {
            msg->text[current_line - 1][0] = '>';
        }
        if (edit_mode && (current_line == 2)) {
            reverse_pixels((char *)msg->text[current_line - 1], 9, 20);
        }
        if (edit_mode && ((current_line == 3) || (current_line == 4) || (current_line == 5))) {
            reverse_pixels((char *)msg->text[current_line - 1], 15 + (4 - (step > 0 ? step + 1 : step)), 20 - (step > 0 ? step + 1 : step));
        }
        break;
    default:
    case HOTTTEXT_PAGE_MAIN: // Main page where HoTT modules can be started
        if (HoTTBridgeSettingsHandle() != NULL) {
            HoTTBridgeSettingsSensorGet(&sensor);
        }
        if (edit_mode) {
            switch (current_line) {
            case 2:
                sensor.VARIO = enable_disable_sensor(sensor.VARIO);
                break;
            case 3:
                sensor.GPS   = enable_disable_sensor(sensor.GPS);
                break;
            case 4:
                sensor.EAM   = enable_disable_sensor(sensor.EAM);
                if (sensor.EAM == HOTTBRIDGESETTINGS_SENSOR_ENABLED) {
                    // no need to emulate General module
                    sensor.GAM = HOTTBRIDGESETTINGS_SENSOR_DISABLED;
                }
                break;
            case 5:
                sensor.GAM = enable_disable_sensor(sensor.GAM);
                if (sensor.GAM == HOTTBRIDGESETTINGS_SENSOR_ENABLED) {
                    // no need to emulate Electric module
                    sensor.EAM = HOTTBRIDGESETTINGS_SENSOR_DISABLED;
                }
                break;
            case 6:
                sensor.ESC = enable_disable_sensor(sensor.ESC);
            }
            HoTTBridgeSettingsSensorSet(&sensor);
            edit_status = HOTTTEXT_EDITSTATUS_DONE;
        }

        // create Main page content
        snprintf(msg->text[1], HOTT_TEXT_COLUMNS, " VARIO module    [%1s]", ((sensor.VARIO == HOTTBRIDGESETTINGS_SENSOR_DISABLED) ? " " : "*")); // line 2
        snprintf(msg->text[2], HOTT_TEXT_COLUMNS, " GPS module      [%1s]", ((sensor.GPS == HOTTBRIDGESETTINGS_SENSOR_DISABLED) ? " " : "*")); // line 3
        snprintf(msg->text[3], HOTT_TEXT_COLUMNS, " ELECTRIC module [%1s]", ((sensor.EAM == HOTTBRIDGESETTINGS_SENSOR_DISABLED) ? " " : "*")); // line 4
        snprintf(msg->text[4], HOTT_TEXT_COLUMNS, " GENERAL module  [%1s]", ((sensor.GAM == HOTTBRIDGESETTINGS_SENSOR_DISABLED) ? " " : "*")); // line 5
        snprintf(msg->text[5], HOTT_TEXT_COLUMNS, " ESC module      [%1s]", ((sensor.ESC == HOTTBRIDGESETTINGS_SENSOR_DISABLED) ? " " : "*")); // line 6
        snprintf(msg->text[6], HOTT_TEXT_COLUMNS, "   Select module    ");
        snprintf(msg->text[7], HOTT_TEXT_COLUMNS, "   to be emulated   ");
        if (current_line > 1) {
            msg->text[current_line - 1][0] = '>';
        }
        // break;
    }

    msg->stop     = HOTT_STOP;

    msg->checksum = calc_checksum((uint8_t *)msg, sizeof(*msg));
    if (edit_status != HOTTTEXT_EDITSTATUS_DONE) {
        edit_status = step;
    }
    return edit_status;
}

/**
 * get next/previous page to display
 */
uint8_t get_page(uint8_t page, bool next)
{
    HoTTBridgeSettingsSensorData sensor;

    if (HoTTBridgeSettingsHandle() != NULL) {
        HoTTBridgeSettingsSensorGet(&sensor);
    }

    if (next) {
        switch (page) {
        case HOTTTEXT_PAGE_MAIN:
            page = HOTTTEXT_PAGE_MAINCONFIG;
            break;
        case HOTTTEXT_PAGE_MAINCONFIG:
            if (sensor.GPS == HOTTBRIDGESETTINGS_SENSOR_ENABLED) {
                page = HOTTTEXT_PAGE_GPSCONFIG;
                break;
            }
        // fall through
        case HOTTTEXT_PAGE_GPSCONFIG:
            if ((sensor.GAM == HOTTBRIDGESETTINGS_SENSOR_ENABLED) ||
                (sensor.EAM == HOTTBRIDGESETTINGS_SENSOR_ENABLED) ||
                (sensor.ESC == HOTTBRIDGESETTINGS_SENSOR_ENABLED)) {
                page = HOTTTEXT_PAGE_BATTERYCONFIG;
                break;
            }
        // fall through
        case HOTTTEXT_PAGE_BATTERYCONFIG:
            if ((sensor.VARIO == HOTTBRIDGESETTINGS_SENSOR_ENABLED) ||
                (sensor.GAM == HOTTBRIDGESETTINGS_SENSOR_ENABLED) ||
                (sensor.GPS == HOTTBRIDGESETTINGS_SENSOR_ENABLED)) {
                page = HOTTTEXT_PAGE_VARIOWARNINGS;
                break;
            }
        // fall through
        case HOTTTEXT_PAGE_VARIOWARNINGS:
            if ((sensor.VARIO == HOTTBRIDGESETTINGS_SENSOR_ENABLED) ||
                (sensor.GAM == HOTTBRIDGESETTINGS_SENSOR_ENABLED) ||
                (sensor.GPS == HOTTBRIDGESETTINGS_SENSOR_ENABLED)) {
                page = HOTTTEXT_PAGE_VARIOLIMITS;
                break;
            }
        // fall through
        case HOTTTEXT_PAGE_VARIOLIMITS:
            if (sensor.GPS == HOTTBRIDGESETTINGS_SENSOR_ENABLED) {
                page = HOTTTEXT_PAGE_GPS;
                break;
            }
        // fall through
        case HOTTTEXT_PAGE_GPS:
            if (sensor.GAM == HOTTBRIDGESETTINGS_SENSOR_ENABLED) {
                page = HOTTTEXT_PAGE_GENERAL;
                break;
            }
        // fall through
        case HOTTTEXT_PAGE_GENERAL:
            if (sensor.EAM == HOTTBRIDGESETTINGS_SENSOR_ENABLED) {
                page = HOTTTEXT_PAGE_ELECTRIC;
                break;
            }
        // fall through
        case HOTTTEXT_PAGE_ELECTRIC:
            if (sensor.ESC == HOTTBRIDGESETTINGS_SENSOR_ENABLED) {
                page = HOTTTEXT_PAGE_ESC;
                break;
            }
        // fall through
        case HOTTTEXT_PAGE_ESC:
            if ((sensor.GAM == HOTTBRIDGESETTINGS_SENSOR_ENABLED) ||
                (sensor.EAM == HOTTBRIDGESETTINGS_SENSOR_ENABLED) ||
                (sensor.ESC == HOTTBRIDGESETTINGS_SENSOR_ENABLED)) {
                page = HOTTTEXT_PAGE_SENSORREDIR;
                break;
            }
        // fall through
        case HOTTTEXT_PAGE_SENSORREDIR:
            break;
        default:
            page = HOTTTEXT_PAGE_MAIN;
        }
    } else {
        switch (page) {
        case HOTTTEXT_PAGE_SENSORREDIR:
            if (sensor.ESC == HOTTBRIDGESETTINGS_SENSOR_ENABLED) {
                page = HOTTTEXT_PAGE_ESC;
                break;
            }
        // fall through
        case HOTTTEXT_PAGE_ESC:
            if (sensor.EAM == HOTTBRIDGESETTINGS_SENSOR_ENABLED) {
                page = HOTTTEXT_PAGE_ELECTRIC;
                break;
            }
        // fall through
        case HOTTTEXT_PAGE_ELECTRIC:
            if (sensor.GAM == HOTTBRIDGESETTINGS_SENSOR_ENABLED) {
                page = HOTTTEXT_PAGE_GENERAL;
                break;
            }
        // fall through
        case HOTTTEXT_PAGE_GENERAL:
            if (sensor.GPS == HOTTBRIDGESETTINGS_SENSOR_ENABLED) {
                page = HOTTTEXT_PAGE_GPS;
                break;
            }
        // fall through
        case HOTTTEXT_PAGE_GPS:
            if ((sensor.VARIO == HOTTBRIDGESETTINGS_SENSOR_ENABLED) ||
                (sensor.GAM == HOTTBRIDGESETTINGS_SENSOR_ENABLED) ||
                (sensor.GPS == HOTTBRIDGESETTINGS_SENSOR_ENABLED)) {
                page = HOTTTEXT_PAGE_VARIOLIMITS;
                break;
            }
        // fall through
        case HOTTTEXT_PAGE_VARIOLIMITS:
            if ((sensor.VARIO == HOTTBRIDGESETTINGS_SENSOR_ENABLED) ||
                (sensor.GAM == HOTTBRIDGESETTINGS_SENSOR_ENABLED) ||
                (sensor.GPS == HOTTBRIDGESETTINGS_SENSOR_ENABLED)) {
                page = HOTTTEXT_PAGE_VARIOWARNINGS;
                break;
            }
        // fall through
        case HOTTTEXT_PAGE_VARIOWARNINGS:
            if ((sensor.GAM == HOTTBRIDGESETTINGS_SENSOR_ENABLED) ||
                (sensor.EAM == HOTTBRIDGESETTINGS_SENSOR_ENABLED) ||
                (sensor.ESC == HOTTBRIDGESETTINGS_SENSOR_ENABLED)) {
                page = HOTTTEXT_PAGE_BATTERYCONFIG;
                break;
            }
        // fall through
        case HOTTTEXT_PAGE_BATTERYCONFIG:
            if (sensor.GPS == HOTTBRIDGESETTINGS_SENSOR_ENABLED) {
                page = HOTTTEXT_PAGE_GPSCONFIG;
                break;
            }
        // fall through
        case HOTTTEXT_PAGE_GPSCONFIG:
            page = HOTTTEXT_PAGE_MAINCONFIG;
            break;
        case HOTTTEXT_PAGE_MAINCONFIG:
            page = HOTTTEXT_PAGE_MAIN;
            break;
        case HOTTTEXT_PAGE_MAIN:
        default:
            page = HOTTTEXT_PAGE_MAIN;
        }
    }
    return page;
}

/**
 * change Hott Warning state
 */
uint8_t enable_disable_warning(uint8_t value)
{
    if (value == HOTTBRIDGESETTINGS_WARNING_DISABLED) {
        value = HOTTBRIDGESETTINGS_WARNING_ENABLED;
    } else {
        value = HOTTBRIDGESETTINGS_WARNING_DISABLED;
    }
    return value;
}

/**
 * change emulated Hott sensor state
 */
uint8_t enable_disable_sensor(uint8_t value)
{
    if (value == HOTTBRIDGESETTINGS_SENSOR_DISABLED) {
        value = HOTTBRIDGESETTINGS_SENSOR_ENABLED;
    } else {
        value = HOTTBRIDGESETTINGS_SENSOR_DISABLED;
    }
    return value;
}

/**
 * get value from redirected sensor
 */
float get_redirect_sensor_value(uint8_t hott_sensor)
{
    HoTTBridgeSettingsSensorRedirectData sensorRedirect;
    HoTTBridgeSettingsSensorRedirectOptions sensor = HOTTBRIDGESETTINGS_SENSORREDIRECT_NONE;
    float value = 0.0f;

    if (HoTTBridgeSettingsHandle() != NULL) {
        HoTTBridgeSettingsSensorRedirectGet(&sensorRedirect);
    }

    switch (hott_sensor) {
    case HOTTBRIDGESETTINGS_SENSORREDIRECT_SPEED:
        sensor = sensorRedirect.Speed;
        break;
    case HOTTBRIDGESETTINGS_SENSORREDIRECT_BATTERY1:
        sensor = sensorRedirect.Battery1;
        break;
    case HOTTBRIDGESETTINGS_SENSORREDIRECT_BATTERY2:
        sensor = sensorRedirect.Battery2;
        break;
    case HOTTBRIDGESETTINGS_SENSORREDIRECT_TEMP1:
        sensor = sensorRedirect.Temp1;
        break;
    case HOTTBRIDGESETTINGS_SENSORREDIRECT_TEMP2:
        sensor = sensorRedirect.Temp2;
        break;
    case HOTTBRIDGESETTINGS_SENSORREDIRECT_PRESSURE:
        sensor = sensorRedirect.Pressure;
        break;
    case HOTTBRIDGESETTINGS_SENSORREDIRECT_RPM:
        sensor = sensorRedirect.Rpm;
        break;
    }

    switch (sensor) {
    case HOTTBRIDGESETTINGS_SENSORREDIRECT_GPSSPEED:
        value = telestate->GPS.Groundspeed;
        break;
    case HOTTBRIDGESETTINGS_SENSORREDIRECT_AIRSPEED:
        value = telestate->Airspeed.TrueAirspeed;
        break;
    case HOTTBRIDGESETTINGS_SENSORREDIRECT_BATTVOLTAGE:
        value = telestate->Battery.Voltage;
        break;
    case HOTTBRIDGESETTINGS_SENSORREDIRECT_GYROTEMPERATURE:
        value = telestate->Gyro.temperature;
        break;
    case HOTTBRIDGESETTINGS_SENSORREDIRECT_BAROTEMPERATURE:
        value = telestate->Baro.Temperature;
        break;
    case HOTTBRIDGESETTINGS_SENSORREDIRECT_TEMPERATURE1:
        value = telestate->Temp.Temperature1;
        break;
    case HOTTBRIDGESETTINGS_SENSORREDIRECT_TEMPERATURE2:
        value = telestate->Temp.Temperature2;
        break;
    case HOTTBRIDGESETTINGS_SENSORREDIRECT_GFORCE:
        value = fabs(telestate->current_G);
        break;
    case HOTTBRIDGESETTINGS_SENSORREDIRECT_NONE:
        value = 0.0f;
    }

    return value;
}

/**
 * get new ADCPin for edited field
 */
int8_t get_newADCPin_value(uint8_t *adcRouting, int8_t from_pin, int8_t value_change)
{
    int8_t newADCPin = from_pin;

    if (value_change > 0) {
        for (int i = from_pin + 1; i < HWSETTINGS_ADCROUTING_NUMELEM; i++) {
            if (adcRouting[i] == HWSETTINGS_ADCROUTING_DISABLED) {
                newADCPin = i;
                break;
            }
        }
    } else if (value_change < 0) {
        for (int i = from_pin - 1; i > ADC_XX_PIN_NOTFOUND - 1; i--) {
            if (i < 0) {
                newADCPin = ADC_XX_PIN_NOTFOUND;
                break;
            }
            if (adcRouting[i] == HWSETTINGS_ADCROUTING_DISABLED) {
                newADCPin = i;
                break;
            }
        }
    }
    return newADCPin;
}

/**
 * get new value for edited field
 */
int16_t get_new_value(int16_t current_value, int8_t value_change, uint8_t step, int16_t min, int16_t max)
{
    uint16_t increment[] = { 1, 10, 100, 1000, 10000 };
    int16_t new_value    = 0;

    new_value = current_value + (value_change * increment[step]);
    if (new_value < min) {
        new_value = min;
    }
    if (new_value > max) {
        new_value = max;
    }
    return new_value;
}

/**
 * store settings to onboard flash
 */
void store_settings(uint8_t page, uint8_t current_line)
{
    switch (page) {
    case HOTTTEXT_PAGE_MAIN:
    case HOTTTEXT_PAGE_VARIOWARNINGS:
    case HOTTTEXT_PAGE_VARIOLIMITS:
    case HOTTTEXT_PAGE_GPS:
    case HOTTTEXT_PAGE_SENSORREDIR:
        UAVObjSave(HoTTBridgeSettingsHandle(), 0);
        break;
    case HOTTTEXT_PAGE_GENERAL:
    case HOTTTEXT_PAGE_ELECTRIC:
    case HOTTTEXT_PAGE_ESC:
        UAVObjSave(HoTTBridgeSettingsHandle(), 0);
        UAVObjSave(FlightBatterySettingsHandle(), 0);
        break;
    case HOTTTEXT_PAGE_BATTERYCONFIG:
        switch (current_line) {
        case 2:
        case 3:
        case 4:
            UAVObjSave(FlightBatterySettingsHandle(), 0);
            break;
        case 5:
        case 6:
            UAVObjSave(HwSettingsHandle(), 0);
            break;
        }
        break;
    case HOTTTEXT_PAGE_GPSCONFIG:
        UAVObjSave(GPSSettingsHandle(), 0);
        break;
    case HOTTTEXT_PAGE_MAINCONFIG:
        switch (current_line) {
        case 2:
            UAVObjSave(RevoSettingsHandle(), 0);
            break;
        case 3:
        case 4:
        case 5:
            UAVObjSave(AttitudeSettingsHandle(), 0);
            break;
        }
    }
}

/**
 * update telemetry data
 * this is called on every telemetry request
 * calling interval is 200ms depending on TX
 * 200ms telemetry request is used as time base for timed calculations (5Hz interval)
 *
 * note : measured around 160ms with a MC20 and adjusted climbratebuffer size from 50 to 62
 * to match the 10s period
 */
void update_telemetrydata()
{
    // update all available data
    if (HoTTBridgeSettingsHandle() != NULL) {
        HoTTBridgeSettingsGet(&telestate->Settings);
    }
    if (AttitudeStateHandle() != NULL) {
        AttitudeStateGet(&telestate->Attitude);
    }
    if (AccelStateHandle() != NULL) {
        AccelStateGet(&telestate->Accel);
    }
    if (BaroSensorHandle() != NULL) {
        BaroSensorGet(&telestate->Baro);
    }
    if (FlightBatteryStateHandle() != NULL) {
        FlightBatteryStateGet(&telestate->Battery);
    }
    if (FlightStatusHandle() != NULL) {
        FlightStatusGet(&telestate->FlightStatus);
    }
    if (GPSPositionSensorHandle() != NULL) {
        GPSPositionSensorGet(&telestate->GPS);
    }
    if (AirspeedStateHandle() != NULL) {
        AirspeedStateGet(&telestate->Airspeed);
    }
    if (GPSTimeHandle() != NULL) {
        GPSTimeGet(&telestate->GPStime);
    }
    if (GyroSensorHandle() != NULL) {
        GyroSensorGet(&telestate->Gyro);
    }
    if (HomeLocationHandle() != NULL) {
        HomeLocationGet(&telestate->Home);
    }
    if (PositionStateHandle() != NULL) {
        PositionStateGet(&telestate->Position);
    }
    if (SystemAlarmsHandle() != NULL) {
        SystemAlarmsGet(&telestate->SysAlarms);
    }
    if (VelocityStateHandle() != NULL) {
        VelocityStateGet(&telestate->Velocity);
    }
    if (TemperatureStateHandle() != NULL) {
        TemperatureStateGet(&telestate->Temp);
    }

    // Make vario less sensitive in +/-VarioSensitivity range
    float sensitivity = (float)telestate->Settings.VarioSensitivity / 100.0f;
    float absVelDown  = fabs(telestate->Velocity.Down);
    if ((absVelDown < sensitivity) && (absVelDown > 0.0f)) {
        telestate->Velocity.Down /= ((sensitivity / absVelDown) * (sensitivity / absVelDown));
    }

    // send actual climbrate value to ring buffer as mm per 0.2s values
    uint8_t n = telestate->climbrate_pointer;
    telestate->climbratebuffer[telestate->climbrate_pointer++] = -telestate->Velocity.Down * 200;
    telestate->climbrate_pointer %= climbratesize;

    // calculate average climbrates in meters per 1, 3 and 10 second(s) based on 200ms interval
    telestate->climbrate1s  = 0;
    telestate->climbrate3s  = 0;
    telestate->climbrate10s = 0;
    for (uint8_t i = 0; i < climbratesize; i++) {
        telestate->climbrate1s  += (i < climbratesize / 10) ? telestate->climbratebuffer[n] : 0;
        telestate->climbrate3s  += (i < (climbratesize / 10) * 3) ? telestate->climbratebuffer[n] : 0;
        telestate->climbrate10s += (i < climbratesize) ? telestate->climbratebuffer[n] : 0;
        n += climbratesize - 1;
        n %= climbratesize;
    }
    telestate->climbrate1s = telestate->climbrate1s / 1000;
    // Increase deadband because radio start beeps with climbrate1s > 0, like +0.01m/s
    // while display shows 0.0
    if ((telestate->climbrate1s > 0) && (telestate->climbrate1s < 0.1f)) {
        telestate->climbrate1s = 0.0f;
    }

    telestate->climbrate3s  = telestate->climbrate3s / 1000;
    telestate->climbrate10s = telestate->climbrate10s / 1000;

    // set altitude offset and clear min/max values when arming
    if ((telestate->FlightStatus.Armed == FLIGHTSTATUS_ARMED_ARMING) || ((telestate->last_armed != FLIGHTSTATUS_ARMED_ARMED) && (telestate->FlightStatus.Armed == FLIGHTSTATUS_ARMED_ARMED))) {
        telestate->min_altitude = 0;
        telestate->max_altitude = 0;
        telestate->max_distance = 0;
    }
    telestate->last_armed   = telestate->FlightStatus.Armed;

    // calculate altitude relative to start position
    telestate->altitude     = -telestate->Position.Down;

    // gps home position and course
    telestate->homedistance = sqrtf(telestate->Position.North * telestate->Position.North + telestate->Position.East * telestate->Position.East);
    telestate->homecourse   = acosf(-telestate->Position.North / telestate->homedistance) / 3.14159265f * 180;
    if (telestate->Position.East > 0) {
        telestate->homecourse = 360 - telestate->homecourse;
    }

    // calculate current 3D speed
    float speed_3d = sqrtf((telestate->Velocity.North * telestate->Velocity.North) + (telestate->Velocity.East * telestate->Velocity.East) + (telestate->Velocity.Down * telestate->Velocity.Down)) * MS_TO_KMH;

    // Normal Acceleration (G unit)
    float nz_alpha = 0.7f;
    telestate->current_G = ((1 - nz_alpha) * (-telestate->Accel.z / 9.81f)) + (telestate->current_G * nz_alpha);

    // check and set min/max values when armed
    // and without receiver input for standalone board used as sensor
    if ((telestate->FlightStatus.Armed == FLIGHTSTATUS_ARMED_ARMED) || ((telestate->SysAlarms.Alarm.Attitude == SYSTEMALARMS_ALARM_OK) && (telestate->SysAlarms.Alarm.Receiver != SYSTEMALARMS_ALARM_OK))) {
        if (telestate->min_altitude > telestate->altitude) {
            telestate->min_altitude = telestate->altitude;
        }
        if (telestate->max_altitude < telestate->altitude) {
            telestate->max_altitude = telestate->altitude;
        }
        if (telestate->max_distance < telestate->homedistance) {
            telestate->max_distance = telestate->homedistance;
        }
        if (telestate->max_speed < speed_3d) {
            telestate->max_speed = speed_3d;
        }
        if (telestate->max_G < telestate->current_G) {
            telestate->max_G = telestate->current_G;
        }
        if (telestate->min_G > telestate->current_G) {
            telestate->min_G = telestate->current_G;
        }

        // temperatures and voltage
        float temp1 = get_redirect_sensor_value(HOTTBRIDGESETTINGS_SENSORREDIRECT_TEMP1);
        float temp2 = get_redirect_sensor_value(HOTTBRIDGESETTINGS_SENSORREDIRECT_TEMP2);
        if (telestate->max_temp1 < temp1) {
            telestate->max_temp1 = temp1;
        }
        if (telestate->max_temp2 < temp2) {
            telestate->max_temp2 = temp2;
        }
        if (telestate->min_voltage > telestate->Battery.Voltage) {
            telestate->min_voltage = telestate->Battery.Voltage;
        }
    }

    // statusline
    const char *txt_unknown          = "unknown";
    const char *txt_manual           = "Manual";
    const char *txt_stabilized1      = "Stabilized1";
    const char *txt_stabilized2      = "Stabilized2";
    const char *txt_stabilized3      = "Stabilized3";
    const char *txt_stabilized4      = "Stabilized4";
    const char *txt_stabilized5      = "Stabilized5";
    const char *txt_stabilized6      = "Stabilized6";
    const char *txt_positionhold     = "PositionHold";
    const char *txt_courselock       = "CourseLock";
    const char *txt_velocityroam     = "VelocityRoam";
    const char *txt_homeleash        = "HomeLeash";
    const char *txt_absoluteposition = "AbsolutePosition";
    const char *txt_returntobase     = "ReturnToBase";
    const char *txt_land = "Land";
    const char *txt_pathplanner      = "PathPlanner";
    const char *txt_poi = "PointOfInterest";
    const char *txt_autocruise       = "AutoCruise";
    const char *txt_autotakeoff      = "AutoTakeOff";
    const char *txt_autotune         = "Autotune";
    const char *txt_disarmed         = "Disarmed";
    const char *txt_arming           = "Arming";
    const char *txt_armed = "Armed";

    const char *txt_flightmode;
    switch (telestate->FlightStatus.FlightMode) {
    case FLIGHTSTATUS_FLIGHTMODE_MANUAL:
        txt_flightmode = txt_manual;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED1:
        txt_flightmode = txt_stabilized1;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED2:
        txt_flightmode = txt_stabilized2;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED3:
        txt_flightmode = txt_stabilized3;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED4:
        txt_flightmode = txt_stabilized4;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED5:
        txt_flightmode = txt_stabilized5;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED6:
        txt_flightmode = txt_stabilized6;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD:
        txt_flightmode = txt_positionhold;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_COURSELOCK:
        txt_flightmode = txt_courselock;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_VELOCITYROAM:
        txt_flightmode = txt_velocityroam;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_HOMELEASH:
        txt_flightmode = txt_homeleash;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_ABSOLUTEPOSITION:
        txt_flightmode = txt_absoluteposition;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_RETURNTOBASE:
        txt_flightmode = txt_returntobase;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_LAND:
        txt_flightmode = txt_land;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_PATHPLANNER:
        txt_flightmode = txt_pathplanner;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_POI:
        txt_flightmode = txt_poi;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_AUTOCRUISE:
        txt_flightmode = txt_autocruise;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_AUTOTAKEOFF:
        txt_flightmode = txt_autotakeoff;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_AUTOTUNE:
        txt_flightmode = txt_autotune;
        break;
    default:
        txt_flightmode = txt_unknown;
    }

    const char *txt_armstate;
    switch (telestate->FlightStatus.Armed) {
    case FLIGHTSTATUS_ARMED_DISARMED:
        txt_armstate = txt_disarmed;
        break;
    case FLIGHTSTATUS_ARMED_ARMING:
        txt_armstate = txt_arming;
        break;
    case FLIGHTSTATUS_ARMED_ARMED:
        txt_armstate = txt_armed;
        break;
    default:
        txt_armstate = txt_unknown;
    }

    // use climbrate pointer for alternate display, without GPS every 5s or 3.3s with speed/dist msg
    uint8_t statusmsg_num = (telestate->Settings.Sensor.GPS == HOTTBRIDGESETTINGS_SENSOR_ENABLED) ? 3 : 2;
    if (telestate->climbrate_pointer < (climbratesize / statusmsg_num)) {
        snprintf(telestate->statusline, sizeof(telestate->statusline), "%12s,%7s", txt_flightmode, txt_armstate);
    } else if (telestate->climbrate_pointer < (climbratesize / statusmsg_num) * 2) {
        snprintf(telestate->statusline, sizeof(telestate->statusline), "MxG % 2d.%d  MnG % 2d.%d",
                 maxd2((int8_t)(telestate->max_G)), (int8_t)fabsf(telestate->max_G * 10) % 10,
                 maxd2((int8_t)(telestate->min_G)), (int8_t)fabsf(telestate->min_G * 10) % 10);
    } else {
        snprintf(telestate->statusline, sizeof(telestate->statusline), "Max %3dkmh Dst %4dm", maxu3((uint16_t)telestate->max_speed), maxu4((uint16_t)telestate->max_distance));
    }
}

/**
 * generate warning beeps or spoken announcements
 */
uint8_t generate_warning()
{
    bool gps_ok = (telestate->SysAlarms.Alarm.GPS == SYSTEMALARMS_ALARM_OK);

    // set warning tone with hardcoded priority
    if ((telestate->Settings.Warning.MinSpeed == HOTTBRIDGESETTINGS_WARNING_ENABLED) &&
        (telestate->Settings.Limit.MinSpeed > telestate->GPS.Groundspeed * MS_TO_KMH) && gps_ok) {
        return HOTT_TONE_A; // minimum speed
    }
    if ((telestate->Settings.Warning.NegDifference2 == HOTTBRIDGESETTINGS_WARNING_ENABLED) &&
        (telestate->Settings.Limit.NegDifference2 > telestate->climbrate3s)) {
        return HOTT_TONE_28; // fast descent
    }
    if ((telestate->Settings.Warning.NegDifference1 == HOTTBRIDGESETTINGS_WARNING_ENABLED) &&
        (telestate->Settings.Limit.NegDifference1 > telestate->climbrate1s) &&
        ((telestate->Settings.Limit.NegDifference1 * 3) < telestate->climbrate3s)) {
        return HOTT_TONE_27; // sudden descent
    }
    if ((telestate->Settings.Warning.MaxDistance == HOTTBRIDGESETTINGS_WARNING_ENABLED) &&
        (telestate->Settings.Limit.MaxDistance < telestate->homedistance) && gps_ok) {
        return HOTT_TONE_D; // maximum distance
    }
    if ((telestate->Settings.Warning.MinSensor1Temp == HOTTBRIDGESETTINGS_WARNING_ENABLED) &&
        (telestate->Settings.Limit.MinSensor1Temp > telestate->Gyro.temperature)) {
        return HOTT_TONE_F; // minimum temperature sensor 1
    }
    if ((telestate->Settings.Warning.MinSensor2Temp == HOTTBRIDGESETTINGS_WARNING_ENABLED) &&
        (telestate->Settings.Limit.MinSensor2Temp > telestate->Baro.Temperature)) {
        return HOTT_TONE_G; // minimum temperature sensor 2
    }
    if ((telestate->Settings.Warning.MaxSensor1Temp == HOTTBRIDGESETTINGS_WARNING_ENABLED) &&
        (telestate->Settings.Limit.MaxSensor1Temp < telestate->Gyro.temperature)) {
        return HOTT_TONE_H; // maximum temperature sensor 1
    }
    if ((telestate->Settings.Warning.MaxSensor2Temp == HOTTBRIDGESETTINGS_WARNING_ENABLED) &&
        (telestate->Settings.Limit.MaxSensor2Temp < telestate->Baro.Temperature)) {
        return HOTT_TONE_I; // maximum temperature sensor 2
    }
    if ((telestate->Settings.Warning.MaxSpeed == HOTTBRIDGESETTINGS_WARNING_ENABLED) &&
        (telestate->Settings.Limit.MaxSpeed < telestate->GPS.Groundspeed * MS_TO_KMH) && gps_ok) {
        return HOTT_TONE_L; // maximum speed
    }
    if ((telestate->Settings.Warning.PosDifference2 == HOTTBRIDGESETTINGS_WARNING_ENABLED) &&
        (telestate->Settings.Limit.PosDifference2 < telestate->climbrate3s)) {
        return HOTT_TONE_35; // fast rise
    }
    if ((telestate->Settings.Warning.PosDifference1 == HOTTBRIDGESETTINGS_WARNING_ENABLED) &&
        (telestate->Settings.Limit.PosDifference1 < telestate->climbrate1s) &&
        ((telestate->Settings.Limit.PosDifference1 * 3) > telestate->climbrate3s)) {
        return HOTT_TONE_36; // sudden rise
    }
    if ((telestate->Settings.Warning.MinHeight == HOTTBRIDGESETTINGS_WARNING_ENABLED) &&
        (telestate->Settings.Limit.MinHeight > telestate->altitude)) {
        return HOTT_TONE_O; // minimum height
    }
    if ((telestate->Settings.Warning.MinPowerVoltage == HOTTBRIDGESETTINGS_WARNING_ENABLED) &&
        (telestate->Settings.Limit.MinPowerVoltage > telestate->Battery.Voltage)) {
        return HOTT_TONE_P; // minimum input voltage
    }
    if ((telestate->Settings.Warning.MaxUsedCapacity == HOTTBRIDGESETTINGS_WARNING_ENABLED) &&
        (telestate->Settings.Limit.MaxUsedCapacity < telestate->Battery.ConsumedEnergy)) {
        return HOTT_TONE_V; // capacity
    }
    if ((telestate->Settings.Warning.MaxCurrent == HOTTBRIDGESETTINGS_WARNING_ENABLED) &&
        (telestate->Settings.Limit.MaxCurrent < telestate->Battery.Current)) {
        return HOTT_TONE_W; // maximum current
    }
    if ((telestate->Settings.Warning.MaxPowerVoltage == HOTTBRIDGESETTINGS_WARNING_ENABLED) &&
        (telestate->Settings.Limit.MaxPowerVoltage < telestate->Battery.Voltage)) {
        return HOTT_TONE_X; // maximum input voltage
    }
    if ((telestate->Settings.Warning.MaxHeight == HOTTBRIDGESETTINGS_WARNING_ENABLED) &&
        (telestate->Settings.Limit.MaxHeight < telestate->altitude)) {
        return HOTT_TONE_Z; // maximum height
    }
    // altitude beeps when crossing altitude limits at 20,40,60,80,100,200,400,600,800 and 1000 meters
    if (telestate->Settings.Warning.AltitudeBeep == HOTTBRIDGESETTINGS_WARNING_ENABLED) {
        // update altitude when checked for beeps
        float last   = telestate->altitude_last;
        float actual = telestate->altitude;
        telestate->altitude_last = telestate->altitude;
        if (((last < 20) && (actual >= 20)) || ((last > 20) && (actual <= 20))) {
            return HOTT_TONE_20M;
        }
        if (((last < 40) && (actual >= 40)) || ((last > 40) && (actual <= 40))) {
            return HOTT_TONE_40M;
        }
        if (((last < 60) && (actual >= 60)) || ((last > 60) && (actual <= 60))) {
            return HOTT_TONE_60M;
        }
        if (((last < 80) && (actual >= 80)) || ((last > 80) && (actual <= 80))) {
            return HOTT_TONE_80M;
        }
        if (((last < 100) && (actual >= 100)) || ((last > 100) && (actual <= 100))) {
            return HOTT_TONE_100M;
        }
        if (((last < 200) && (actual >= 200)) || ((last > 200) && (actual <= 200))) {
            return HOTT_TONE_200M;
        }
        if (((last < 400) && (actual >= 400)) || ((last > 400) && (actual <= 400))) {
            return HOTT_TONE_400M;
        }
        if (((last < 600) && (actual >= 600)) || ((last > 600) && (actual <= 600))) {
            return HOTT_TONE_600M;
        }
        if (((last < 800) && (actual >= 800)) || ((last > 800) && (actual <= 800))) {
            return HOTT_TONE_800M;
        }
        if (((last < 1000) && (actual >= 1000)) || ((last > 1000) && (actual <= 1000))) {
            return HOTT_TONE_1000M;
        }
    }

    // there is no warning
    return 0;
}

/**
 * reverse pixels
 */
char *reverse_pixels(char *line, uint8_t from_char, uint8_t to_char)
{
    for (int i = from_char; i < to_char; i++) {
        if (line[i] == 0) {
            line[i] = (uint8_t)(0x80 + 0x20);
        } else {
            line[i] = (0x80 + line[i]);
        }
    }
    return line;
}

/**
 * calculate checksum of data buffer
 */
uint8_t calc_checksum(uint8_t *data, uint16_t size)
{
    uint16_t sum = 0;

    for (int i = 0; i < size; i++) {
        sum += data[i];
    }
    return sum;
}

/**
 * scale float value with scale and offset to unsigned byte
 */
uint8_t scale_float2uint8(float value, float scale, float offset)
{
    uint16_t temp = (uint16_t)roundf(value * scale + offset);
    uint8_t result;

    result = (uint8_t)temp & 0xff;
    return result;
}

/**
 * scale float value with scale and offset to signed byte (int8_t)
 */
int8_t scale_float2int8(float value, float scale, float offset)
{
    int8_t result = (int8_t)roundf(value * scale + offset);

    return result;
}

/**
 * scale float value with scale and offset to word
 */
uword_t scale_float2uword(float value, float scale, float offset)
{
    uint16_t temp = (uint16_t)roundf(value * scale + offset);
    uword_t result;

    result.l = (uint8_t)temp & 0xff;
    result.h = (uint8_t)(temp >> 8) & 0xff;
    return result;
}

/**
 * convert dword gps value into HoTT gps format and write result to given pointers
 */
void convert_long2gps(int32_t value, uint8_t *dir, uword_t *min, uword_t *sec)
{
    // convert gps decigrad value into degrees, minutes and seconds
    uword_t temp;
    uint32_t absvalue = abs(value);
    uint16_t degrees  = (absvalue / 10000000);
    uint32_t seconds  = (absvalue - degrees * 10000000) * 6;
    uint16_t minutes  = seconds / 1000000;

    seconds %= 1000000;
    seconds  = seconds / 100;
    uint16_t degmin = degrees * 100 + minutes;
    // write results
    *dir     = (value < 0) ? 1 : 0;
    temp.l   = (uint8_t)degmin & 0xff;
    temp.h   = (uint8_t)(degmin >> 8) & 0xff;
    *min     = temp;
    temp.l   = (uint8_t)seconds & 0xff;
    temp.h   = (uint8_t)(seconds >> 8) & 0xff;
    *sec     = temp;
}

#endif // PIOS_INCLUDE_HOTT_BRIDGE
/**
 * @}
 * @}
 */
