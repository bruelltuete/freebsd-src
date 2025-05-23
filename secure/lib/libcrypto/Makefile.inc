
.include <bsd.own.mk>

# OpenSSL version used for manual page generation
OPENSSL_VER=	1.1.1w
OPENSSL_DATE=	2023-09-11

LCRYPTO_SRC=	${SRCTOP}/crypto/openssl
LCRYPTO_DOC=	${LCRYPTO_SRC}/doc

CFLAGS+=	-I${LCRYPTO_SRC}
CFLAGS+=	-I${LCRYPTO_SRC}/include

.include <bsd.endian.mk>

.if ${TARGET_ENDIANNESS} == 1234
CFLAGS+=	-DL_ENDIAN
.elif ${TARGET_ENDIANNESS} == 4321
CFLAGS+=	-DB_ENDIAN
.endif

.if ${MACHINE_CPUARCH} == "aarch64" || ${MACHINE_CPUARCH} == "amd64" || \
    ${MACHINE_CPUARCH} == "arm" || ${MACHINE_CPUARCH} == "i386"
ASM_${MACHINE_CPUARCH}=
.elif ${MACHINE_ARCH} == "powerpc" || ${MACHINE_ARCH} == "powerpc64" || \
    ${MACHINE_ARCH} == "powerpc64le"
ASM_${MACHINE_ARCH}=
.endif

.if defined(ASM_${MACHINE_CPUARCH}) || defined(ASM_${MACHINE_ARCH})
CFLAGS+=	-DOPENSSL_CPUID_OBJ
.if defined(ASM_aarch64)
CFLAGS+=	-DOPENSSL_BN_ASM_MONT
CFLAGS+=	-DSHA1_ASM -DSHA256_ASM -DSHA512_ASM
CFLAGS+=	-DKECCAK1600_ASM
CFLAGS+=	-DVPAES_ASM
CFLAGS+=	-DECP_NISTZ256_ASM
CFLAGS+=	-DPOLY1305_ASM
.elif defined(ASM_amd64)
CFLAGS+=	-DOPENSSL_IA32_SSE2
CFLAGS+=	-DOPENSSL_BN_ASM_MONT -DOPENSSL_BN_ASM_MONT5
CFLAGS+=	-DOPENSSL_BN_ASM_GF2m
CFLAGS+=	-DSHA1_ASM -DSHA256_ASM -DSHA512_ASM
CFLAGS+=	-DKECCAK1600_ASM
CFLAGS+=	-DRC4_ASM
CFLAGS+=	-DMD5_ASM
CFLAGS+=	-DVPAES_ASM
CFLAGS+=	-DGHASH_ASM
CFLAGS+=	-DECP_NISTZ256_ASM -DX25519_ASM
CFLAGS+=	-DPADLOCK_ASM
CFLAGS+=	-DPOLY1305_ASM
.elif defined(ASM_arm)
CFLAGS+=	-DOPENSSL_BN_ASM_MONT -DOPENSSL_BN_ASM_GF2m
CFLAGS+=	-DSHA1_ASM -DSHA256_ASM -DSHA512_ASM
CFLAGS+=	-DKECCAK1600_ASM
CFLAGS+=	-DBSAES_ASM
CFLAGS+=	-DGHASH_ASM
CFLAGS+=	-DECP_NISTZ256_ASM
CFLAGS+=	-DPOLY1305_ASM
.elif defined(ASM_i386)
CFLAGS+=	-DOPENSSL_IA32_SSE2
CFLAGS+=	-DOPENSSL_BN_ASM_PART_WORDS -DOPENSSL_BN_ASM_MONT
CFLAGS+=	-DOPENSSL_BN_ASM_GF2m
CFLAGS+=	-DSHA1_ASM -DSHA256_ASM -DSHA512_ASM
CFLAGS+=	-DRC4_ASM
CFLAGS+=	-DMD5_ASM
CFLAGS+=	-DRMD160_ASM
CFLAGS+=	-DVPAES_ASM
CFLAGS+=	-DWHIRLPOOL_ASM
CFLAGS+=	-DGHASH_ASM
CFLAGS+=	-DECP_NISTZ256_ASM
CFLAGS+=	-DPADLOCK_ASM
CFLAGS+=	-DPOLY1305_ASM
.elif defined(ASM_powerpc)
CFLAGS+=	-DOPENSSL_BN_ASM_MONT
CFLAGS+=	-DAES_ASM
CFLAGS+=	-DVPAES_ASM
CFLAGS+=	-DSHA1_ASM
CFLAGS+=	-DSHA256_ASM
CFLAGS+=	-DSHA512_ASM
CFLAGS+=	-DPOLY1305_ASM
.elif defined(ASM_powerpc64)
CFLAGS+=	-DOPENSSL_BN_ASM_MONT
CFLAGS+=	-DAES_ASM
CFLAGS+=	-DVPAES_ASM
CFLAGS+=	-DSHA1_ASM
CFLAGS+=	-DSHA256_ASM
CFLAGS+=	-DSHA512_ASM
CFLAGS+=	-DPOLY1305_ASM
CFLAGS+=	-DECP_NISTZ256_ASM
CFLAGS+=	-DX25519_ASM
CFLAGS+=	-DKECCAK1600_ASM
.elif defined(ASM_powerpc64le)
CFLAGS+=	-DOPENSSL_BN_ASM_MONT
CFLAGS+=	-DAES_ASM
CFLAGS+=	-DVPAES_ASM
CFLAGS+=	-DSHA1_ASM
CFLAGS+=	-DSHA256_ASM
CFLAGS+=	-DSHA512_ASM
CFLAGS+=	-DPOLY1305_ASM
CFLAGS+=	-DECP_NISTZ256_ASM
CFLAGS+=	-DX25519_ASM
CFLAGS+=	-DKECCAK1600_ASM
.endif
.endif

.if defined(LIB)
CFLAGS+=	-DOPENSSLDIR="\"/etc/ssl\""
CFLAGS+=	-DENGINESDIR="\"${LIBDIR}/engines\""
.endif

CFLAGS+=	-DNDEBUG

MANDIR=		${SHAREDIR}/openssl/man/man

.for pcfile in ${PCFILES}
${pcfile}:	${pcfile}.in
	sed -e 's,@openssl_ver@,${OPENSSL_VER},g' ${.ALLSRC} > ${.TARGET}
.endfor
CLEANFILES+=	${PCFILES}
