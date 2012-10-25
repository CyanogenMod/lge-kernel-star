/* *
*drivers/startablet/star_cam_pmic.h
* Copyright (C) 2010 LGE Inc.
* Author:Hyunsu Choi
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/


#ifndef STAR_CAM_PMIC_H
#define STAR_CAM_PMIC_H

#define BSSQ_CAM_PMIC "bssq_cam_pmic"  //20110525 calvin.hwang@lge.com Camsensor ks1001 merge

extern int star_cam_Main_power_on(void);
extern int star_cam_VT_power_on(void);
extern int star_cam_power_off(void);
#endif 

