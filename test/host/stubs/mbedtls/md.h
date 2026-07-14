#pragma once
/* Minimal: the packet module is compiled with auth disabled in host tests, so
 * these are only needed for the #include to parse; none are called. */
#include <stddef.h>
typedef enum { MBEDTLS_MD_NONE = 0, MBEDTLS_MD_SHA256 } mbedtls_md_type_t;
typedef struct mbedtls_md_info_t mbedtls_md_info_t;
typedef struct { void *p; } mbedtls_md_context_t;
const mbedtls_md_info_t *mbedtls_md_info_from_type(mbedtls_md_type_t t);
void mbedtls_md_init(mbedtls_md_context_t *c);
void mbedtls_md_free(mbedtls_md_context_t *c);
int mbedtls_md_setup(mbedtls_md_context_t *c, const mbedtls_md_info_t *i, int hmac);
int mbedtls_md_hmac_starts(mbedtls_md_context_t *c, const unsigned char *k, size_t kl);
int mbedtls_md_hmac_update(mbedtls_md_context_t *c, const unsigned char *in, size_t il);
int mbedtls_md_hmac_finish(mbedtls_md_context_t *c, unsigned char *out);
int mbedtls_md_hmac_reset(mbedtls_md_context_t *c);
