#	@(#)Makefile	8.1 (Berkeley) 6/6/93

.include <src.opts.mk>

PACKAGE=	runtime

PROG=		xinstall
PROGNAME=	install
SRCS=		xinstall.c getid.c
MAN=		install.1

.PATH:		${SRCTOP}/contrib/mtree
CFLAGS+=	-I${SRCTOP}/contrib/mtree
CFLAGS+=	-I${SRCTOP}/lib/libnetbsd

LIBADD=		md
CFLAGS+=	-DWITH_MD5 -DWITH_RIPEMD160

HAS_TESTS=
SUBDIR.${MK_TESTS}+= tests

.include <bsd.prog.mk>
