
.include <src.opts.mk>
.include "${SRCTOP}/secure/ssh.mk"

CONFS=	moduli sshd_config
CONFSDIR=	/etc/ssh
PROG=	sshd
SRCS=	sshd.c \
	platform-listen.c \
	servconf.c sshpty.c srclimit.c groupaccess.c auth2-methods.c
PACKAGE=	ssh

MAN=	sshd.8 sshd_config.5

# Don't rebuild based on moduli.c
moduli: .MADE

LIBADD=	ssh util

.if ${MK_GSSAPI} != "no" && ${MK_KERBEROS_SUPPORT} != "no"
CFLAGS+= -include krb5_config.h
SRCS+=	krb5_config.h
LIBADD+=	gssapi_krb5 gssapi krb5
.endif

.if ${MK_TCP_WRAPPERS} != "no"
CFLAGS+=	-DLIBWRAP=1
LIBADD+=	wrap
.endif

LIBADD+=	crypto

.include <bsd.prog.mk>

.PATH:	${SSHDIR}
