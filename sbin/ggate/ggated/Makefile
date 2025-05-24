
.PATH: ${.CURDIR:H}/shared

PROG=	ggated
MAN=	ggated.8
SRCS=	ggated.c ggate.c
PACKAGE=	ggate

LIBADD=	pthread util geom

CFLAGS+= -I${.CURDIR:H}/shared

.include <bsd.prog.mk>
