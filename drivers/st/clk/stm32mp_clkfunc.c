/*
 * Copyright (c) 2017-2018, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <libfdt.h>
#include <platform_def.h>
#include <stm32_gpio.h>
#include <stm32mp_clkfunc.h>
#include <stm32mp_common.h>
#include <stm32mp_dt.h>

#define DT_STGEN_COMPAT		"st,stm32-stgen"
#define DT_UART_COMPAT		"st,stm32h7-uart"

/*******************************************************************************
 * This function returns the RCC node in the device tree.
 ******************************************************************************/
int fdt_get_rcc_node(void *fdt)
{
	return fdt_node_offset_by_compatible(fdt, -1, DT_RCC_CLK_COMPAT);
}

/*******************************************************************************
 * This function reads the rcc base address.
 * It reads the value indicated inside the device tree.
 * Returns address on success, and 0 on failure.
 ******************************************************************************/
uint32_t fdt_rcc_read_addr(void)
{
	int node;
	void *fdt;
	const fdt32_t *cuint;

	if (fdt_get_address(&fdt) == 0) {
		return 0;
	}

	node = fdt_get_rcc_node(fdt);
	if (node < 0) {
		return 0;
	}

	cuint = fdt_getprop(fdt, node, "reg", NULL);
	if (cuint == NULL) {
		return 0;
	}

	return fdt32_to_cpu(*cuint);
}

/*******************************************************************************
 * This function reads a series of parameters in rcc-clk section.
 * It reads the values indicated inside the device tree, from property name.
 * The number of parameters is also indicated as entry parameter.
 * Returns 0 on success, and a negative FDT/ERRNO error code on failure.
 * On success, values are stored at the second parameter address.
 ******************************************************************************/
int fdt_rcc_read_uint32_array(const char *prop_name,
			      uint32_t *array, uint32_t count)
{
	int node;
	void *fdt;

	if (fdt_get_address(&fdt) == 0) {
		return -ENOENT;
	}

	node = fdt_get_rcc_node(fdt);
	if (node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	return fdt_read_uint32_array(node, prop_name, array, count);
}

/*******************************************************************************
 * This function reads a property rcc-clk section.
 * It reads the values indicated inside the device tree, from property name.
 * Returns dflt_value if property is not found, and a property value on
 * success.
 ******************************************************************************/
uint32_t fdt_rcc_read_uint32_default(const char *prop_name, uint32_t dflt_value)
{
	int node;
	void *fdt;

	if (fdt_get_address(&fdt) == 0) {
		return dflt_value;
	}

	node = fdt_node_offset_by_compatible(fdt, -1, DT_RCC_CLK_COMPAT);
	if (node < 0) {
		return dflt_value;
	}

	return fdt_read_uint32_default(node, prop_name, dflt_value);
}

/*******************************************************************************
 * This function gets the subnode offset in rcc-clk section from its name.
 * It reads the values indicated inside the device tree.
 * Returns offset on success, and a negative FDT/ERRNO error code on failure.
 ******************************************************************************/
int fdt_rcc_subnode_offset(const char *name)
{
	int node, subnode;
	void *fdt;

	if (fdt_get_address(&fdt) == 0) {
		return -ENOENT;
	}

	node = fdt_get_rcc_node(fdt);
	if (node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	subnode = fdt_subnode_offset(fdt, node, name);
	if (subnode <= 0) {
		return -FDT_ERR_NOTFOUND;
	}

	return subnode;
}

/*******************************************************************************
 * This function gets the pointer to a rcc-clk property from its name.
 * It reads the values indicated inside the device tree.
 * Length of the property is stored in the second parameter.
 * Returns pointer on success, and NULL value on failure.
 ******************************************************************************/
const fdt32_t *fdt_rcc_read_prop(const char *prop_name, int *lenp)
{
	const fdt32_t *cuint;
	int node, len;
	void *fdt;

	if (fdt_get_address(&fdt) == 0) {
		return NULL;
	}

	node = fdt_get_rcc_node(fdt);
	if (node < 0) {
		return NULL;
	}

	cuint = fdt_getprop(fdt, node, prop_name, &len);
	if (cuint == NULL) {
		return NULL;
	}

	*lenp = len;
	return cuint;
}

/*******************************************************************************
 * This function gets the secure status for rcc node.
 * It reads secure-status in device tree.
 * Returns true if rcc is available from secure world, false if not.
 ******************************************************************************/
bool fdt_get_rcc_secure_status(void)
{
	int node;
	void *fdt;

	if (fdt_get_address(&fdt) == 0) {
		return false;
	}

	node = fdt_get_rcc_node(fdt);
	if (node < 0) {
		return false;
	}

	return !!(fdt_get_status(node) & DT_SECURE);
}

/*******************************************************************************
 * This function reads the stgen base address.
 * It reads the value indicated inside the device tree.
 * Returns address on success, and NULL value on failure.
 ******************************************************************************/
uintptr_t fdt_get_stgen_base(void)
{
	int node;
	const fdt32_t *cuint;
	void *fdt;

	if (fdt_get_address(&fdt) == 0) {
		return 0;
	}

	node = fdt_node_offset_by_compatible(fdt, -1, DT_STGEN_COMPAT);
	if (node < 0) {
		return 0;
	}

	cuint = fdt_getprop(fdt, node, "reg", NULL);
	if (cuint == NULL) {
		return 0;
	}

	return fdt32_to_cpu(*cuint);
}

/*******************************************************************************
 * This function gets the clock ID of the given node.
 * It reads the value indicated inside the device tree.
 * Returns ID on success, and a negative FDT/ERRNO error code on failure.
 ******************************************************************************/
int fdt_get_clock_id(int node)
{
	const fdt32_t *cuint;
	void *fdt;

	if (fdt_get_address(&fdt) == 0) {
		return -ENOENT;
	}

	cuint = fdt_getprop(fdt, node, "clocks", NULL);
	if (cuint == NULL) {
		return -FDT_ERR_NOTFOUND;
	}

	cuint++;
	return (int)fdt32_to_cpu(*cuint);
}

/*******************************************************************************
 * This function gets the clock ID of the given node using clock-names.
 * It reads the value indicated inside the device tree.
 * Returns ID on success, and a negative FDT/ERRNO error code on failure.
 ******************************************************************************/
int fdt_get_clock_id_by_name(int node, const char *name)
{
	const fdt32_t *cuint;
	void *fdt;
	int index, len;

	if (fdt_get_address(&fdt) == 0) {
		return -ENOENT;
	}

	index = fdt_stringlist_search(fdt, node, "clock-names", name);
	if (index < 0) {
		return index;
	}

	cuint = fdt_getprop(fdt, node, "clocks", &len);
	if (cuint == NULL) {
		return -FDT_ERR_NOTFOUND;
	}

	if ((index * (int)sizeof(uint32_t)) > len) {
		return -FDT_ERR_BADVALUE;
	}

	cuint += (index << 1) + 1;
	return (int)fdt32_to_cpu(*cuint);
}

/*******************************************************************************
 * This function gets the frequency of the specified uart instance.
 * From this instance, all the uarts nodes in DT are parsed, and the register
 * base is compared to the instance. If match between these two values, then
 * the clock source is read from the DT and we deduce the frequency.
 * Returns clock frequency on success, 0 value on failure.
 ******************************************************************************/
unsigned long fdt_get_uart_clock_freq(uintptr_t instance)
{
	int node;
	void *fdt;

	if (fdt_get_address(&fdt) == 0) {
		return 0;
	}

	/* Check for UART nodes */
	node = fdt_node_offset_by_compatible(fdt, -1, DT_UART_COMPAT);
	while (node != -FDT_ERR_NOTFOUND) {
		const fdt32_t *cuint;

		cuint = fdt_getprop(fdt, node, "reg", NULL);
		if (cuint == NULL)
			goto next;

		if ((uintptr_t)fdt32_to_cpu(*cuint) == instance) {
			unsigned long clk_id;

			cuint = fdt_getprop(fdt, node, "clocks", NULL);
			if (cuint == NULL)
				goto next;

			cuint++;
			clk_id = (unsigned long)(fdt32_to_cpu(*cuint));

			return stm32mp_clk_get_rate(clk_id);
		}
next:
		node = fdt_node_offset_by_compatible(fdt, node, DT_UART_COMPAT);
	}

	return 0;
}
