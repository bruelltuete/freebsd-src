.PATH:  ${SRCTOP}/sys/ofed/drivers/infiniband/core

KMOD=	ibcore
SRCS= \
	ib_addr.c \
	ib_agent.c \
	ib_cache.c \
	ib_cm.c \
	ib_cma.c \
	ib_cq.c \
	ib_device.c \
	ib_fmr_pool.c \
	ib_iwcm.c \
	ib_iwpm_msg.c \
	ib_iwpm_util.c \
	ib_mad.c \
	ib_mad_rmpp.c \
	ib_multicast.c \
	ib_packer.c \
	ib_roce_gid_mgmt.c \
	ib_sa_query.c \
	ib_smi.c \
	ib_sysfs.c \
	ib_ucm.c \
	ib_ucma.c \
	ib_ud_header.c \
	ib_umem.c \
	ib_user_mad.c \
	ib_uverbs_cmd.c \
	ib_uverbs_main.c \
	ib_uverbs_marshall.c \
	ib_verbs.c
SRCS+=	${LINUXKPI_GENSRCS}
SRCS+=	opt_inet.h opt_inet6.h

CFLAGS+= -I${SRCTOP}/sys/ofed/include
CFLAGS+= -I${SRCTOP}/sys/ofed/include/uapi
CFLAGS+= ${LINUXKPI_INCLUDES}
CFLAGS+= -DINET6 -DINET -DCONFIG_INFINIBAND_USER_MEM

EXPORT_SYMS=	YES

.include <bsd.kmod.mk>

CWARNFLAGS+= -Wno-cast-qual -Wno-pointer-arith
