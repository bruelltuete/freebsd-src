/*-
 * Copyright (c) 2018-2020 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * This software was developed by BAE Systems, the University of Cambridge
 * Computer Laboratory, and Memorial University under DARPA/AFRL contract
 * FA8650-15-C-7558 ("CADETS"), as part of the DARPA Transparent Computing
 * (TC) research program.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm64/coresight/coresight.h>
#include <arm64/coresight/coresight_etm4x.h>

#include "coresight_if.h"

static struct ofw_compat_data compat_data[] = {
	{ "arm,coresight-etm4x",		1 },
	{ NULL,					0 }
};

static int
etm_fdt_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "AArch64 Embedded Trace Macrocell");

	return (BUS_PROBE_DEFAULT);
}

static int
etm_fdt_attach(device_t dev)
{
	struct etm_softc *sc;

	sc = device_get_softc(dev);
	sc->pdata = coresight_fdt_get_platform_data(dev);

	return (etm_attach(dev));
}

static device_method_t etm_fdt_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		etm_fdt_probe),
	DEVMETHOD(device_attach,	etm_fdt_attach),
	DEVMETHOD_END
};

DEFINE_CLASS_1(etm, etm_fdt_driver, etm_fdt_methods,
    sizeof(struct etm_softc), etm_driver);

static devclass_t etm_fdt_devclass;

EARLY_DRIVER_MODULE(etm, simplebus, etm_fdt_driver, etm_fdt_devclass,
    0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);
