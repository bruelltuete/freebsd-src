
.include <src.opts.mk>

PROG_CXX=	clang

SRCDIR=		clang/tools/driver
SRCS+=		cc1_main.cpp
SRCS+=		cc1as_main.cpp
SRCS+=		cc1gen_reproducer_main.cpp
SRCS+=		clang-driver.cpp
SRCS+=		driver.cpp

.if ${MK_SHARED_TOOLCHAIN} == "no"
NO_SHARED?= yes

.endif

LINKS=	${BINDIR}/clang ${BINDIR}/clang++ \
	${BINDIR}/clang ${BINDIR}/clang-cpp
MLINKS=	clang.1 clang++.1 \
	clang.1 clang-cpp.1
.if ${MK_CLANG_IS_CC} != "no"
SCRIPTS=CC.sh
SCRIPTSNAME=CC

LINKS+=	${BINDIR}/clang ${BINDIR}/cc \
	${BINDIR}/clang ${BINDIR}/c++ \
	${BINDIR}/clang ${BINDIR}/cpp
MLINKS+= clang.1 cc.1 \
	clang.1 c++.1 \
	clang.1 CC.1 \
	clang.1 cpp.1
.endif

# Ensure we don't add CFLAGS.clang when using GCC
NO_TARGET_FLAGS=

.include "../clang.prog.mk"
