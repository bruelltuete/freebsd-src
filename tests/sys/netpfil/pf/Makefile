
PACKAGE=	tests

TESTSDIR=       ${TESTSBASE}/sys/netpfil/pf
TESTS_SUBDIRS+=	ioctl

ATF_TESTS_SH+=	altq \
		anchor \
		checksum \
		dup \
		forward \
		fragmentation \
		get_state \
		icmp \
		icmp6 \
		killstate \
		macro \
		map_e \
		names \
		nat \
		pass_block \
		pfsync	\
		proxy \
		ridentifier \
		route_to \
		rules_counter \
		sctp \
		set_skip \
		set_tos \
		snmp \
		src_track \
		syncookie \
		synproxy \
		table \
		tcp \
		tos

ATF_TESTS_PYTEST+=	frag6.py
ATF_TESTS_PYTEST+=	sctp.py

# Tests reuse jail names and so cannot run in parallel.
TEST_METADATA+=	is_exclusive=true

${PACKAGE}FILES+=	\
			bsnmpd.conf \
			CVE-2019-5597.py \
			CVE-2019-5598.py \
			echo_inetd.conf \
			fragcommon.py \
			frag-overindex.py \
			frag-overlimit.py \
			frag-overreplace.py \
			pfsync_defer.py \
			utils.subr

${PACKAGE}FILESMODE_bsnmpd.conf=		0555
${PACKAGE}FILESMODE_CVE-2019-5597.py=	0555
${PACKAGE}FILESMODE_CVE-2019-5598.py=	0555
${PACKAGE}FILESMODE_fragcommon.py=	0555
${PACKAGE}FILESMODE_frag-overindex.py=	0555
${PACKAGE}FILESMODE_frag-overlimit.py=	0555
${PACKAGE}FILESMODE_frag-overreplace.py=	0555
${PACKAGE}FILESMODE_pfsync_defer.py=	0555

.include <bsd.test.mk>
