/**
 ******************************************************************************
 * @addtogroup OpenPilotSystem OpenPilot System
 * @{
 * @addtogroup OpenPilotLibraries OpenPilot System Libraries
 * @brief OpenPilot System libraries are available to all OP modules.
 * @{
 * @file       alarms.c
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2016.
 *             The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Library for setting and clearing system alarms
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

#include <openpilot.h>
#include "inc/alarms.h"

// Private constants
#ifndef PIOS_ALARM_GRACETIME
// alarm cannot be turned off for at least 1000 milliseconds
// to prevent event system overload through flapping alarms
        #define PIOS_ALARM_GRACETIME 1000
#endif // PIOS_ALARM_GRACETIME

// Private types

// Private variables
static xSemaphoreHandle lock;
static volatile uint16_t lastAlarmChange[SYSTEMALARMS_ALARM_NUMELEM] = { 0 }; // this deliberately overflows every 2^16 milliseconds to save memory

// Private functions
static int32_t hasSeverity(SystemAlarmsAlarmOptions severity);

/**
 * Initialize the alarms library
 */
int32_t AlarmsInitialize(void)
{
    SystemAlarmsInitialize();

    lock = xSemaphoreCreateRecursiveMutex();
    // do not change the default states of the alarms, let the init code generated by the uavobjectgenerator handle that
    // AlarmsClearAll();
    // AlarmsDefaultAll();
    return 0;
}

/**
 * Set an alarm
 * @param alarm The system alarm to be modified
 * @param severity The alarm severity
 * @return 0 if success, -1 if an error
 */
int32_t AlarmsSet(SystemAlarmsAlarmElem alarm, SystemAlarmsAlarmOptions severity)
{
    SystemAlarmsAlarmData alarms;

    // Check that this is a valid alarm
    if (alarm >= SYSTEMALARMS_ALARM_NUMELEM) {
        return -1;
    }

    // Lock
    xSemaphoreTakeRecursive(lock, portMAX_DELAY);

    // Read alarm and update its severity only if it was changed
    SystemAlarmsAlarmGet(&alarms);
    uint16_t flightTime = (uint16_t)xTaskGetTickCount() * (uint16_t)portTICK_RATE_MS; // this deliberately overflows every 2^16 milliseconds to save memory
    if (((uint16_t)(flightTime - lastAlarmChange[alarm]) > PIOS_ALARM_GRACETIME &&
         SystemAlarmsAlarmToArray(alarms)[alarm] != severity)
        || SystemAlarmsAlarmToArray(alarms)[alarm] < severity) {
        SystemAlarmsAlarmToArray(alarms)[alarm] = severity;
        lastAlarmChange[alarm] = flightTime;
        SystemAlarmsAlarmSet(&alarms);
    }

    // Release lock
    xSemaphoreGiveRecursive(lock);
    return 0;
}

/**
 * Set an Extended Alarm
 * @param alarm The system alarm to be modified
 * @param severity The alarm severity
 * @param status The Extended alarm status field
 * @param subStatus The Extended alarm substatus field
 * @return 0 if success, -1 if an error
 */
int32_t ExtendedAlarmsSet(SystemAlarmsAlarmElem alarm,
                          SystemAlarmsAlarmOptions severity,
                          SystemAlarmsExtendedAlarmStatusOptions status,
                          uint8_t subStatus)
{
    SystemAlarmsData alarms;

    // Check that this is a valid alarm
    if (alarm >= SYSTEMALARMS_EXTENDEDALARMSTATUS_NUMELEM) {
        return -1;
    }

    // Lock
    xSemaphoreTakeRecursive(lock, portMAX_DELAY);

    // Read alarm and update its severity only if it was changed
    SystemAlarmsGet(&alarms);
    uint16_t flightTime = (uint16_t)xTaskGetTickCount() * (uint16_t)portTICK_RATE_MS; // this deliberately overflows every 2^16 milliseconds to save memory
    if (((uint16_t)(flightTime - lastAlarmChange[alarm]) > PIOS_ALARM_GRACETIME &&
         SystemAlarmsAlarmToArray(alarms.Alarm)[alarm] != severity)
        || SystemAlarmsAlarmToArray(alarms.Alarm)[alarm] < severity) {
        SystemAlarmsExtendedAlarmStatusToArray(alarms.ExtendedAlarmStatus)[alarm] = status;
        SystemAlarmsExtendedAlarmSubStatusToArray(alarms.ExtendedAlarmSubStatus)[alarm] = subStatus;
        SystemAlarmsAlarmToArray(alarms.Alarm)[alarm] = severity;
        lastAlarmChange[alarm] = flightTime;
        SystemAlarmsSet(&alarms);
    }

    // Release lock
    xSemaphoreGiveRecursive(lock);
    return 0;
}

/**
 * Get an alarm
 * @param alarm The system alarm to be read
 * @return Alarm severity
 */
SystemAlarmsAlarmOptions AlarmsGet(SystemAlarmsAlarmElem alarm)
{
    SystemAlarmsAlarmData alarms;

    // Check that this is a valid alarm
    if (alarm >= SYSTEMALARMS_ALARM_NUMELEM) {
        return 0;
    }

    // Read alarm
    SystemAlarmsAlarmGet(&alarms);
    return SystemAlarmsAlarmToArray(alarms)[alarm];
}

/**
 * Set an alarm to it's default value
 * @param alarm The system alarm to be modified
 * @return 0 if success, -1 if an error
 */
int32_t AlarmsDefault(SystemAlarmsAlarmElem alarm)
{
    return AlarmsSet(alarm, SYSTEMALARMS_ALARM_DEFAULT);
}

/**
 * Default all alarms
 */
void AlarmsDefaultAll()
{
    for (uint32_t n = 0; n < SYSTEMALARMS_ALARM_NUMELEM; ++n) {
        AlarmsDefault(n);
    }
}

/**
 * Clear an alarm
 * @param alarm The system alarm to be modified
 * @return 0 if success, -1 if an error
 */
int32_t AlarmsClear(SystemAlarmsAlarmElem alarm)
{
    if (alarm < SYSTEMALARMS_EXTENDEDALARMSTATUS_NUMELEM) {
        return ExtendedAlarmsSet(alarm, SYSTEMALARMS_ALARM_OK, SYSTEMALARMS_EXTENDEDALARMSTATUS_NONE, 0);
    } else {
        return AlarmsSet(alarm, SYSTEMALARMS_ALARM_OK);
    }
}

/**
 * Clear all alarms
 */
void AlarmsClearAll()
{
    for (uint32_t n = 0; n < SYSTEMALARMS_ALARM_NUMELEM; ++n) {
        AlarmsClear(n);
    }
}

/**
 * Check if there are any alarms with the given or higher severity
 * @return 0 if no alarms are found, 1 if at least one alarm is found
 */
int32_t AlarmsHasWarnings()
{
    return hasSeverity(SYSTEMALARMS_ALARM_WARNING);
}

/**
 * Check if there are any alarms with error or higher severity
 * @return 0 if no alarms are found, 1 if at least one alarm is found
 */
int32_t AlarmsHasErrors()
{
    return hasSeverity(SYSTEMALARMS_ALARM_ERROR);
}


/**
 * Check if there are any alarms with critical or higher severity
 * @return 0 if no alarms are found, 1 if at least one alarm is found
 */
int32_t AlarmsHasCritical()
{
    return hasSeverity(SYSTEMALARMS_ALARM_CRITICAL);
}


/**
 * Check if there are any alarms with the given or higher severity
 * @return 0 if no alarms are found, 1 if at least one alarm is found
 */
static int32_t hasSeverity(SystemAlarmsAlarmOptions severity)
{
    SystemAlarmsAlarmData alarms;

    // Lock
    xSemaphoreTakeRecursive(lock, portMAX_DELAY);

    // Read alarms
    SystemAlarmsAlarmGet(&alarms);

    // Go through alarms and check if any are of the given severity or higher
    for (uint32_t n = 0; n < SYSTEMALARMS_ALARM_NUMELEM; ++n) {
        if (SystemAlarmsAlarmToArray(alarms)[n] >= severity) {
            xSemaphoreGiveRecursive(lock);
            return 1;
        }
    }

    // If this point is reached then no alarms found
    xSemaphoreGiveRecursive(lock);
    return 0;
}
/**
 * Get the highest alarm severity
 * @return
 */
SystemAlarmsAlarmOptions AlarmsGetHighestSeverity()
{
    SystemAlarmsAlarmData alarmsData;
    SystemAlarmsAlarmOptions highest = SYSTEMALARMS_ALARM_UNINITIALISED;

    // Lock
    xSemaphoreTakeRecursive(lock, portMAX_DELAY);

    // Read alarms
    SystemAlarmsAlarmGet(&alarmsData);

    // Go through alarms and find the highest severity
    uint32_t n = 0;
    while (n < SYSTEMALARMS_ALARM_NUMELEM && highest != SYSTEMALARMS_ALARM_CRITICAL) {
        if (SystemAlarmsAlarmToArray(alarmsData)[n] > highest) {
            highest = SystemAlarmsAlarmToArray(alarmsData)[n];
        }
        n++;
    }

    xSemaphoreGiveRecursive(lock);
    return highest;
}


__attribute__((unused)) static const char *const systemalarms_severity_names[] = {
    [SYSTEMALARMS_ALARM_UNINITIALISED] = "UNINITIALISED",
    [SYSTEMALARMS_ALARM_OK]       = "OK",
    [SYSTEMALARMS_ALARM_WARNING]  = "WARNING",
    [SYSTEMALARMS_ALARM_CRITICAL] = "CRITICAL",
    [SYSTEMALARMS_ALARM_ERROR]    = "ERROR"
};

static const char *const systemalarms_alarm_names[] = {
    [SYSTEMALARMS_ALARM_SYSTEMCONFIGURATION] = "CONFIG",
    [SYSTEMALARMS_ALARM_BOOTFAULT]     = "BOOT",
    [SYSTEMALARMS_ALARM_OUTOFMEMORY]   = "MEM",
    [SYSTEMALARMS_ALARM_STACKOVERFLOW] = "STACK",
    [SYSTEMALARMS_ALARM_CPUOVERLOAD]   = "CPU",
    [SYSTEMALARMS_ALARM_EVENTSYSTEM]   = "EVENT",
    [SYSTEMALARMS_ALARM_TELEMETRY]     = "TELEMETRY",
    [SYSTEMALARMS_ALARM_RECEIVER] = "INPUT",
    [SYSTEMALARMS_ALARM_MANUALCONTROL] = "MANUAL",
    [SYSTEMALARMS_ALARM_ACTUATOR]      = "ACTUATOR",
    [SYSTEMALARMS_ALARM_ATTITUDE]      = "ATTI",
    [SYSTEMALARMS_ALARM_SENSORS]       = "SENSOR",
    [SYSTEMALARMS_ALARM_MAGNETOMETER]  = "MAG",
    [SYSTEMALARMS_ALARM_AIRSPEED]      = "AIRSPD",
    [SYSTEMALARMS_ALARM_STABILIZATION] = "STAB",
    [SYSTEMALARMS_ALARM_NAV] = "NAV",
    [SYSTEMALARMS_ALARM_GUIDANCE]      = "GUIDANCE",
    [SYSTEMALARMS_ALARM_PATHPLAN]      = "PLAN",
    [SYSTEMALARMS_ALARM_BATTERY]       = "BATT",
    [SYSTEMALARMS_ALARM_FLIGHTTIME]    = "TIME",
    [SYSTEMALARMS_ALARM_I2C] = "I2C",
    [SYSTEMALARMS_ALARM_GPS] = "GPS",
};

static const char *const systemalarms_extendedalarmstatus_names[] = {
    [SYSTEMALARMS_EXTENDEDALARMSTATUS_REBOOTREQUIRED] = "CFG:REBOOT",
    [SYSTEMALARMS_EXTENDEDALARMSTATUS_FLIGHTMODE]     = "CFG:FLIGHTMODE",
    [SYSTEMALARMS_EXTENDEDALARMSTATUS_UNSUPPORTEDCONFIG_ONESHOT] = "CFG:ONESHOT",
    [SYSTEMALARMS_EXTENDEDALARMSTATUS_BADTHROTTLEORCOLLECTIVEINPUTRANGE] = "CFG:THR-COL",
};

size_t AlarmString(SystemAlarmsData *alarm, char *buffer, size_t buffer_size, SystemAlarmsAlarmOptions level, SystemAlarmsAlarmOptions *highestSeverity)
{
    size_t pos = 0;

    PIOS_STATIC_ASSERT(NELEMENTS(systemalarms_alarm_names) == SYSTEMALARMS_ALARM_NUMELEM);

    for (unsigned severity = SYSTEMALARMS_ALARM_ERROR; severity >= level; --severity) {
        // should we prepend severity level here? No, not for now.

        for (unsigned i = 0; i < SYSTEMALARMS_ALARM_NUMELEM; ++i) {
            if ((SystemAlarmsAlarmToArray(alarm->Alarm)[i] == severity)
                && (systemalarms_alarm_names[i])) {
                if (highestSeverity) { // they are already sorted by severity as we are processing in specific order
                    *highestSeverity = severity;
                    highestSeverity  = 0;
                }

                // in which case should we dig into extended alarm status?
                // looks like SYSTEMALARMS_ALARM_SYSTEMCONFIGURATION sets most of the extended alarms
                // except SYSTEMALARMS_ALARM_BOOTFAULT which also sets SYSTEMALARMS_EXTENDEDALARMSTATUS_REBOOTREQUIRED

                const char *current_msg = systemalarms_alarm_names[i];

                switch (i) {
                case SYSTEMALARMS_ALARM_SYSTEMCONFIGURATION:
                    if (alarm->ExtendedAlarmStatus.SystemConfiguration < NELEMENTS(systemalarms_extendedalarmstatus_names)) {
                        current_msg = systemalarms_extendedalarmstatus_names[alarm->ExtendedAlarmStatus.SystemConfiguration];
                    }
                    break;

                case SYSTEMALARMS_ALARM_BOOTFAULT:
                    if (alarm->ExtendedAlarmStatus.BootFault < NELEMENTS(systemalarms_extendedalarmstatus_names)) {
                        current_msg = systemalarms_extendedalarmstatus_names[alarm->ExtendedAlarmStatus.BootFault];
                    }
                    break;
                }

                int current_len = strlen(current_msg);

                if ((pos + current_len + 1) > buffer_size) {
                    break;
                }

                memcpy(buffer + pos, current_msg, current_len);

                pos += current_len;

                buffer[pos++] = ',';
            }
        }
    }

    if (pos > 0) {
        --pos; // get rid of that trailing separator.
    }

    buffer[pos] = 0;

    return pos; // return length of the string in buffer. Actual bytes written is +1
}

/**
 * @}
 * @}
 */
