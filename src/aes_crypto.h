#ifndef _AES_CRYPTO_H
#define _AES_CRYPTO_H

#include <openssl/conf.h>
#include <openssl/evp.h>
#include <openssl/err.h>
#include <string.h>
#include <sys/types.h>
#include <stdint.h>

typedef union {
    uint8_t  bytes[16];
    uint16_t words[8];
    uint32_t dwords[4];
    uint64_t ddwords[2];
} data128_t;

typedef union {
    uint8_t  bytes[32];
    uint16_t words[16];
    uint32_t dwords[8];
    uint64_t ddwords[4];
} data256_t;

struct ctx_data
{
    EVP_CIPHER_CTX *ctx;

    uint64_t  cseq;
    data256_t key;
    data256_t salt;
    data128_t iv;

#define AES_IV_BITS 128
#define AES_IV_LEN  (AES_IV_BITS/8)

#define AES_KEY_BITS 256
#define AES_KEY_LEN  (AES_KEY_BITS/8)

#define AES_SALT_LEN  (AES_KEY_BITS/8)
};

#define DEFAULT_SALT_BYTE 0x11

void init_crypto(void);
void destroy_crypto(void);

int init_ctx_data(struct ctx_data* ctx, unsigned char *key, size_t key_len, unsigned char *salt, size_t salt_len);
void destroy_ctx_data(struct ctx_data* ctx);
int update_iv(struct ctx_data* ctx, uint64_t session, uint64_t cseq);
int aes_encrypt(struct ctx_data *ctx, unsigned char *plaintext, int plaintext_len, unsigned char *ciphertext);
int aes_decrypt(struct ctx_data *ctx, unsigned char *ciphertext, int ciphertext_len, unsigned char *plaintext);


#endif //_AES_CRYPTO_H
