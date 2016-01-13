#ifndef _MESH_SECURITY_TOOLBOX_CCM_H_
#define _MESH_SECURITY_TOOLBOX_CCM_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct _mesh_security_ccm_context
{
	uint8_t key[16];
}mesh_security_ccm_context;



int mesh_security_ccm_encrypt_and_tag( mesh_security_ccm_context *ctx, size_t length,
                         const unsigned char *iv, size_t iv_len,
                         const unsigned char *add, size_t add_len,
                         const unsigned char *input, unsigned char *output,
                         unsigned char *tag, size_t tag_len );

int mesh_security_ccm_auth_decrypt( mesh_security_ccm_context *ctx, size_t length,
                      const unsigned char *iv, size_t iv_len,
                      const unsigned char *add, size_t add_len,
                      const unsigned char *input, unsigned char *output,
                      const unsigned char *tag, size_t tag_len );

#endif



