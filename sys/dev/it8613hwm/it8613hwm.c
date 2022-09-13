/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Johannes Totz
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

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <dev/superio/superio.h>

#include <machine/bus.h>
#include <machine/resource.h>

struct it8613hwm_softc {
	struct resource*	ioport_res;
	int			ioport_rid;

	struct mtx		lock;
};

static int it8613hwm_get_temp_sysctl(SYSCTL_HANDLER_ARGS)
{
	device_t dev = (device_t) arg1;
	struct it8613hwm_softc* sc = device_get_softc(dev);
	int tempindex = arg2;
	int tempkelvin = 0;
	uint8_t tempcelsius;
	int8_t tempoffset;

	if (tempindex >= 0 && tempindex < 3)
	{
		mtx_lock(&sc->lock);

		bus_write_1(sc->ioport_res, 0, 0x29 + tempindex);
		tempcelsius = bus_read_1(sc->ioport_res, 1);

		/*
		 * The offset registers are: 0x56, 0x57, 0x59.
		 * (Register 0x58 is the vendor id for the chip.)
		 */
		bus_write_1(sc->ioport_res, 0, 0x56 + (tempindex < 2 ? tempindex : 3));
		tempoffset = bus_read_1(sc->ioport_res, 1);

		tempkelvin = tempcelsius - tempoffset + 273;

		mtx_unlock(&sc->lock);
	}

	return sysctl_handle_int(oidp, &tempkelvin, 0, req);
}

static int it8613hwm_get_fan_sysctl(SYSCTL_HANDLER_ARGS)
{
	device_t dev = (device_t) arg1;
	struct it8613hwm_softc* sc = device_get_softc(dev);
	int fanindex = arg2;
	unsigned int fanspeedrpm = 0;

	if (fanindex >= 0 && fanindex < 3)
	{
		mtx_lock(&sc->lock);

		/* High bits of fan tachometer. */
		bus_write_1(sc->ioport_res, 0, 0x18 + fanindex);
		fanspeedrpm = bus_read_1(sc->ioport_res, 1) << 8;
		/* Low bits of fan tachometer. */
		bus_write_1(sc->ioport_res, 0, 0x0D + fanindex);
		fanspeedrpm |= bus_read_1(sc->ioport_res, 1);

		/*
		 * The fan tacho has a 22.5 kHz clock and produces 2 pulses per
		 * revolution.
		 */
		if (fanspeedrpm > 0)
			fanspeedrpm = (22500 * 60) / (fanspeedrpm * 2);

		mtx_unlock(&sc->lock);
	}

	return sysctl_handle_int(oidp, &fanspeedrpm, 0, req);
}

static int it8613hwm_probe(device_t dev)
{
	if (superio_vendor(dev) != SUPERIO_VENDOR_ITE)
		return (ENXIO);
	if (superio_get_type(dev) != SUPERIO_DEV_HWM)
		return (ENXIO);
	if (superio_devid(dev) != 0x8613)
		return (ENXIO);

	device_set_desc(dev, "Hardware monitor on ITE SuperIO");
	return (BUS_PROBE_DEFAULT);
}

static int it8613hwm_attach(device_t dev)
{
	struct it8613hwm_softc* sc = device_get_softc(dev);
	struct sysctl_ctx_list* ctx;
	int error;
	uint16_t iobase;
	uint8_t chipid;

	/*
	 * The hardware monitor sits at offset 5 in the chip's io address space.
	 * We'll claim two ports: first (iobase + 5 + 0) is the chip's register
	 * selector; the second (iobase + 5 + 1) is the data register.
	 */
	iobase = superio_get_iobase(dev);
	error = bus_set_resource(dev, SYS_RES_IOPORT, sc->ioport_rid,
	    iobase + 5, 2);
	if (error != 0)
	{
		device_printf(dev, "bus_set_resource failed\n");
		return (ENXIO);
	}
	sc->ioport_res = bus_alloc_resource_any(dev, SYS_RES_IOPORT,
	    &sc->ioport_rid, RF_ACTIVE);
	if (sc->ioport_res == NULL)
	{
		device_printf(dev, "bus_alloc_resource_any failed\n");
		return (ENXIO);
	}

	/* Check hardware monitor vendor id. This should be 0x90 for the it8613. */
	bus_write_1(sc->ioport_res, 0, 0x58);
	chipid = bus_read_1(sc->ioport_res, 1);
	if (chipid != 0x90)
	{
		device_printf(dev, "unexpected chip id: actual %i != expected %i\n",
		    chipid, 0x90);
		bus_release_resource(dev, SYS_RES_IOPORT, sc->ioport_rid,
		    sc->ioport_res);
		return (ENXIO);
	}

	mtx_init(&sc->lock, device_get_nameunit(dev), "it8613hwm", MTX_DEF);

	ctx = device_get_sysctl_ctx(dev);
	/* Gets us "dev.it8613hwm.0.temperature0" etc */
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "temperature0", CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    dev, 0, it8613hwm_get_temp_sysctl, "IK0", "Temperature sensor 0");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "temperature1", CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    dev, 1, it8613hwm_get_temp_sysctl, "IK0", "Temperature sensor 1");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "temperature2", CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    dev, 2, it8613hwm_get_temp_sysctl, "IK0", "Temperature sensor 2");

	/* Gets us "dev.it8613hwm.0.fan0" etc */
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "fan0", CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    dev, 0, it8613hwm_get_fan_sysctl, "I", "RPM of fan 0");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "fan1", CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    dev, 1, it8613hwm_get_fan_sysctl, "I", "RPM of fan 1");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "fan2", CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    dev, 2, it8613hwm_get_fan_sysctl, "I", "RPM of fan 2");

	return (0);
}

static int it8613hwm_detach(device_t dev)
{
	struct it8613hwm_softc* sc = device_get_softc(dev);

	bus_release_resource(dev, SYS_RES_IOPORT, sc->ioport_rid,
	    sc->ioport_res);
	mtx_destroy(&sc->lock);

	return (0);
}

static device_method_t it8613hwm_methods[] = {
	/* Methods from the device interface */
	DEVMETHOD(device_probe,		it8613hwm_probe),
	DEVMETHOD(device_attach,	it8613hwm_attach),
	DEVMETHOD(device_detach,	it8613hwm_detach),

	/* Terminate method list */
	{ 0, 0 }
};

static driver_t it8613hwm_driver = {
	"it8613hwm",
	it8613hwm_methods,
	sizeof(struct it8613hwm_softc)
};

static devclass_t it8613hwm_devclass;

DRIVER_MODULE(it8613hwm, superio, it8613hwm_driver, it8613hwm_devclass, NULL,
    NULL);
MODULE_DEPEND(it8613hwm, superio, 1, 1, 1);
MODULE_VERSION(it8613hwm, 1);
