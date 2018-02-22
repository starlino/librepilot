/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_QMC5883L QMC5883L Functions
 * @brief Deals with the hardware interface to the QMC5883L magnetometer
 * @{
 *
 * @file       pios_qmc5883l.h
 * @author     The LibrePilot Project, http://www.librepilot.org, Copyright (C) 2018
 * @brief      QMC5883L functions header.
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

#ifndef PIOS_QMC5883L_H
#define PIOS_QMC5883L_H

struct pios_qmc5883l_dev;
typedef struct pios_qmc5883l_dev *pios_qmc5883l_dev_t;

extern pios_qmc5883l_dev_t PIOS_QMC5883L_Init(uint32_t i2c_device);


#endif /* PIOS_QMC5883L_H */
