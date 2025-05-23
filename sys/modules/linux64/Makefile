
.PATH: ${SRCTOP}/sys/compat/linux ${SRCTOP}/sys/${MACHINE}/linux
.if ${MACHINE_CPUARCH} == "amd64"
.PATH: ${SRCTOP}/sys/x86/linux
.endif

KMOD=	linux64
SRCS=	linux_dummy_machdep.c \
	linux_elf64.c \
	linux_event.c \
	linux_file.c \
	linux_fork.c \
	linux_futex.c \
	linux_getcwd.c \
	linux_ioctl.c \
	linux_ipc.c \
	linux_machdep.c \
	linux_misc.c \
	linux_ptrace.c \
	linux_rseq.c \
	linux_signal.c \
	linux_socket.c \
	linux_stats.c \
	linux_syscalls.c \
	linux_sysctl.c \
	linux_sysent.c \
	linux_sysvec.c \
	linux_time.c \
	linux_vdso.c \
	linux_timer.c \
	opt_ktrace.h \
	opt_inet6.h \
	opt_posix.h \
	bus_if.h \
	device_if.h \
	vnode_if.h \
	linux_support.s
.if ${MACHINE_CPUARCH} == "amd64"
SRCS+=	linux_dummy_x86.c
.endif
DPSRCS=	assym.inc linux_genassym.c

# XXX: for assym.inc
SRCS+=  opt_kstack_pages.h opt_nfs.h opt_hwpmc_hooks.h

CLEANFILES=	linux_assym.h linux_genassym.o linux_locore.o \
		genassym.o linux_vdso_gtod.o linux_vdso.so.o

OBJS=	linux_vdso.so

linux_assym.h: linux_genassym.o
	sh ${SYSDIR}/kern/genassym.sh linux_genassym.o > ${.TARGET}

.if ${MACHINE_CPUARCH} == "amd64"
VDSOFLAGS=-mcmodel=small -msoft-float
VDSODEPS=linux_vdso_gettc_x86.inc
.elif ${MACHINE_CPUARCH} == "aarch64"
# The Linux uses tiny memory model, but our ld does not know about
# some of relocation types which is generated by cc
VDSOFLAGS=-mgeneral-regs-only -mcmodel=small -ffixed-x18
.endif

linux_locore.o: linux_assym.h assym.inc
	${CC} -c -x assembler-with-cpp -DLOCORE \
	-fPIC -pipe -O2 -Werror ${VDSOFLAGS} \
	-nostdinc -fasynchronous-unwind-tables \
	-fno-omit-frame-pointer -foptimize-sibling-calls \
	-fno-stack-protector -I. -I${SYSDIR} -I${SRCTOP}/include \
	    ${.IMPSRC} -o ${.TARGET}

linux_vdso_gtod.o: linux_vdso_gtod.inc ${VDSODEPS}
	${CC} -c -fPIC -pipe -O2 -Werror ${VDSOFLAGS} \
	-nostdinc -fasynchronous-unwind-tables \
	-fno-omit-frame-pointer -foptimize-sibling-calls \
	-fno-stack-protector -I. -I${SYSDIR} -I${SRCTOP}/include \
	    ${.IMPSRC} -o ${.TARGET}

linux_vdso.so.o: linux_locore.o linux_vdso_gtod.o
	${LD} --shared --eh-frame-hdr -soname=linux-vdso.so.1 \
	--no-undefined --hash-style=both -warn-common -nostdlib \
	--strip-debug -s --build-id=sha1 -Bsymbolic \
	-T${SRCTOP}/sys/${MACHINE}/linux/linux_vdso.lds.s \
	-o ${.TARGET} ${.ALLSRC:M*.o}

.if ${MACHINE_CPUARCH} == "aarch64"
OBJCOPY_TARGET=--output-target elf64-littleaarch64 --binary-architecture aarch64
.elif ${MACHINE_CPUARCH} == "amd64"
OBJCOPY_TARGET=--output-target elf64-x86-64 --binary-architecture i386:x86-64
.else
.error ${MACHINE_CPUARCH} not yet supported by linux64
.endif

linux_vdso.so: linux_vdso.so.o
	${OBJCOPY} --input-target binary ${OBJCOPY_TARGET} \
	    linux_vdso.so.o ${.TARGET}
	${STRIPBIN} -N _binary_linux_vdso_so_o_size ${.TARGET}

linux_support.o: assym.inc linux_assym.h
	${CC} -c -x assembler-with-cpp -DLOCORE ${CFLAGS} \
	    ${.IMPSRC} -o ${.TARGET}

linux_genassym.o: offset.inc
	${CC} -c ${CFLAGS:N-flto:N-fno-common} -fcommon ${.IMPSRC}

.if !defined(KERNBUILDDIR)
.warning Building Linuxulator outside of a kernel does not make sense
.endif

EXPORT_SYMS=	YES

.include <bsd.kmod.mk>
