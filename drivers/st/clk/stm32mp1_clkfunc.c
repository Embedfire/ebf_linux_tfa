/*
 * Copyright (c) 2017-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <dt-bindings/clock/stm32mp1-clksrc.h>
#include <errno.h>
#include <libfdt.h>
#include <platform_def.h>
#include <stm32_gpio.h>
#include <stm32mp_clkfunc.h>
#include <stm32mp1_clk.h>
#include <stm32mp1_clkfunc.h>

const char *stm32mp_osc_node_label[NB_OSC] = {
	[_LSI] = "clk-lsi",
	[_LSE] = "clk-lse",
	[_HSI] = "clk-hsi",
	[_HSE] = "clk-hse",
	[_CSI] = "clk-csi",
	[_I2S_CKIN] = "i2s_ckin"
};

/*******************************************************************************
 * This function reads the frequency of an oscillator from its name.
 * It reads the value indicated inside the device tree.
 * Returns 0 on success, and a negative FDT/ERRNO error code on failure.
 * On success, value is stored in the second parameter.
 ******************************************************************************/
int fdt_osc_read_freq(const char *name, uint32_t *freq)
{
	int node, subnode;
	void *fdt;

	if (fdt_get_address(&fdt) == 0) {
		return -ENOENT;
	}

	node = fdt_path_offset(fdt, "/clocks");
	if (node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	fdt_for_each_subnode(subnode, fdt, node) {
		const char *cchar;
		int ret;

		cchar = fdt_get_name(fdt, subnode, &ret);
		if (cchar == NULL) {
			return ret;
		}

		if (strncmp(cchar, name, (size_t)ret) == 0) {
			const fdt32_t *cuint;

			if (fdt_get_status(subnode) == DT_DISABLED) {
				goto exit;
			}

			cuint = fdt_getprop(fdt, subnode, "clock-frequency",
					    &ret);
			if (cuint == NULL) {
				return ret;
			}

			*freq = fdt32_to_cpu(*cuint);

			return 0;
		}
	}

exit:
	/* Oscillator not found or disabled, freq=0 */
	*freq = 0;
	return 0;
}

/*******************************************************************************
 * This function checks the presence of an oscillator property from its id.
 * The search is done inside the device tree.
 * Returns true/false regarding search result.
 ******************************************************************************/
bool fdt_osc_read_bool(enum stm32mp_osc_id osc_id, const char *prop_name)
{
	int node, subnode;
	void *fdt;

	if (fdt_get_address(&fdt) == 0) {
		return false;
	}

	if (osc_id >= NB_OSC) {
		return false;
	}

	node = fdt_path_offset(fdt, "/clocks");
	if (node < 0) {
		return false;
	}

	fdt_for_each_subnode(subnode, fdt, node) {
		const char *cchar;
		int ret;

		cchar = fdt_get_name(fdt, subnode, &ret);
		if (cchar == NULL) {
			return false;
		}

		if (strncmp(cchar, stm32mp_osc_node_label[osc_id],
			    (size_t)ret) != 0) {
			continue;
		}

		if (fdt_getprop(fdt, subnode, prop_name, NULL) != NULL) {
			return true;
		}
	}

	return false;
}

/*******************************************************************************
 * This function reads a value of a oscillator property from its id.
 * Returns value on success, and a default value if property not found.
 * Default value is passed as parameter.
 ******************************************************************************/
uint32_t fdt_osc_read_uint32_default(enum stm32mp_osc_id osc_id,
				     const char *prop_name, uint32_t dflt_value)
{
	int node, subnode;
	void *fdt;

	if (fdt_get_address(&fdt) == 0) {
		return dflt_value;
	}

	if (osc_id >= NB_OSC) {
		return dflt_value;
	}

	node = fdt_path_offset(fdt, "/clocks");
	if (node < 0) {
		return dflt_value;
	}

	fdt_for_each_subnode(subnode, fdt, node) {
		const char *cchar;
		int ret;

		cchar = fdt_get_name(fdt, subnode, &ret);
		if (cchar == NULL) {
			return dflt_value;
		}

		if (strncmp(cchar, stm32mp_osc_node_label[osc_id],
			    (size_t)ret) != 0) {
			continue;
		}

		return fdt_read_uint32_default(subnode, prop_name, dflt_value);
	}

	return dflt_value;
}


/*******************************************************************************
 * This function get interrupt name.
 * It reads the values indicated the enabling status.
 * Returns 0 if success, and a negative value else.
 ******************************************************************************/
int fdt_rcc_enable_it(const char *name)
{
	int node = fdt_get_rcc_node();

	if (node < 0) {
		return -ENODEV;
	}

	return stm32_gic_enable_spi(node, name);
}
