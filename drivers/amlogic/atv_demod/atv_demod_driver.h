/*
 * drivers/amlogic/atv_demod/atv_demod_driver.h
 *
 * Copyright (C) 2017 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#ifndef __ATV_DEMOD_DRIVER_H__
#define __ATV_DEMOD_DRIVER_H__


#include <media/v4l2-device.h>
#include "drivers/media/dvb-core/dvb_frontend.h"
#include "atv_demod_v4l2.h"


struct aml_atvdemod_device {
	char *name;
	struct class cls;
	struct device *dev;

	unsigned int tuner_num;
	int tuner_cur;
	struct aml_tuner *tuners;

	unsigned int if_freq;
	unsigned int if_inv;
	u64 std;
	unsigned int audmode;
	unsigned int sound_mode;
	int fre_offset;

	struct pinctrl *agc_pin;
	const char *pin_name;

	struct v4l2_frontend v4l2_fe;
	bool analog_attached;
	bool tuner_attached;

	int irq;

	void __iomem *demod_reg_base;
	void __iomem *audiodemod_reg_base;
	void __iomem *hiu_reg_base;
	void __iomem *periphs_reg_base;
	void __iomem *audio_reg_base;

	unsigned int reg_23cf; /* IIR filter */
	int btsc_sap_mode; /*0: off 1:monitor 2:auto */

#define ATVDEMOD_STATE_IDEL  0
#define ATVDEMOD_STATE_WORK  1
#define ATVDEMOD_STATE_SLEEP 2
	int atvdemod_state;

	int (*demod_reg_write)(unsigned int reg, unsigned int val);
	int (*demod_reg_read)(unsigned int reg, unsigned int *val);

	int (*audio_reg_write)(unsigned int reg, unsigned int val);
	int (*audio_reg_read)(unsigned int reg, unsigned int *val);

	int (*hiu_reg_write)(unsigned int reg, unsigned int val);
	int (*hiu_reg_read)(unsigned int reg, unsigned int *val);

	int (*periphs_reg_write)(unsigned int reg, unsigned int val);
	int (*periphs_reg_read)(unsigned int reg, unsigned int *val);
};

extern struct aml_atvdemod_device *amlatvdemod_devp;

extern int aml_atvdemod_attach_demod(struct aml_atvdemod_device *dev);
extern int aml_atvdemod_attach_tuner(struct aml_atvdemod_device *dev);

#endif /* __ATV_DEMOD_DRIVER_H__ */
