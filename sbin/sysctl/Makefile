#	@(#)Makefile	8.1 (Berkeley) 6/6/93
.include <src.opts.mk>

PACKAGE=runtime
CONFS=	sysctl.conf
PROG=	sysctl
WARNS?=	3
MAN=	sysctl.8

.if ${MK_JAIL} != "no" && !defined(RESCUE)
CFLAGS+=	-DJAIL
LIBADD+=	jail
.endif

.include <bsd.prog.mk>
