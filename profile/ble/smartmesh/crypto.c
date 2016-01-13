

#include "crypto.h"
#include "aes.h"
#include "aes_cmac.h"

typedef uint8_t __u8;
typedef uint16_t __u16;
typedef uint32_t __u32;

#define ALG_SET_KEY                     1
#define ALG_SET_IV                      2
#define ALG_SET_OP                      3

#define ALG_OP_DECRYPT                  0
#define ALG_OP_ENCRYPT                  1

#define PF_ALG		38	/* Algorithm sockets.  */
#define AF_ALG		PF_ALG

#define SOL_ALG		279

#define ssize_t size_t

/* Maximum message length that can be passed to aes_cmac */
#define CMAC_MSG_MAX	80


/*
 * Security function e
 *
 * Security function e generates 128-bit encryptedData from a 128-bit key
 * and 128-bit plaintextData using the AES-128-bit block cypher:
 *
 *   encryptedData = e(key, plaintextData)
 *
 * The most significant octet of key corresponds to key[0], the most
 * significant octet of plaintextData corresponds to in[0] and the
 * most significant octet of encryptedData corresponds to out[0].
 *
 */
int  bt_crypto_e(const uint8_t key[16],
	uint8_t plaintext[16], uint8_t encrypted[16])
{
	AES128_ECB_encrypt(plaintext, key, encrypted);
	return 0;
}


