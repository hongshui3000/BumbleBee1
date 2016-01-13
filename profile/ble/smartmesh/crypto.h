#ifndef _CRYPTO_H_
#define _CRYPTO_H_

#include <stdbool.h>
#include <stdint.h>

/*bt encryption functions*/

/*===========legacy================================*/
/*aes-128*/
int bt_crypto_e(const uint8_t key[16],
	 uint8_t plaintext[16], uint8_t encrypted[16]);

#endif
