
PACKAGE=	tests

TESTSDIR=       ${TESTSBASE}/sys/netipsec/tunnel

ATF_TESTS_SH+=	empty \
		aes_cbc_128_hmac_sha1 \
		aes_cbc_256_hmac_sha2_256 \
		aes_gcm_128 \
		aes_gcm_256 \
		aesni_aes_cbc_128_hmac_sha1 \
		aesni_aes_cbc_256_hmac_sha2_256 \
		aesni_aes_gcm_128 \
		aesni_aes_gcm_256

# Each test uses the same names for its jails, so they must be run serially.
TEST_METADATA+=	is_exclusive=true

${PACKAGE}FILES+=	utils.subr

.include <bsd.test.mk>
