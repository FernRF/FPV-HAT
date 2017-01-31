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


void handleErrors(void)
{
    ERR_print_errors_fp(stderr);
    abort();
}


int init_ctx_data(struct ctx_data* ctx, unsigned char *key, size_t key_len, unsigned char *salt, size_t salt_len)
{
    if (ctx == NULL)
        return;

    if(!(ctx->ctx = EVP_CIPHER_CTX_new())) {
        handleErrors();
        return -1;
    }

    memset(&ctx->iv.bytes[0], 0x00, AES_IV_LEN);
    memset(&ctx->key.bytes[0], 0xff, AES_KEY_LEN);
    memset(&ctx->salt.bytes[0],  DEFAULT_SALT_BYTE, AES_SALT_LEN);

    memcpy(&ctx->key.bytes[0], key, key_len > AES_KEY_LEN ? AES_KEY_LEN : key_len);
    if (salt && salt_len > 0)
        memcpy(&ctx->salt.bytes[0], salt, salt_len > AES_SALT_LEN ? AES_SALT_LEN : salt_len);
}

int update_iv(struct ctx_data* ctx, uint64_t session, uint64_t cseq)
{
    //IV = (k_s * 2^16) XOR (SSRC * 2^64) XOR (i * 2^16)
    ctx->iv.ddwords[0] = ctx->salt.ddwords[0] ^ cseq ^ session;
    ctx->iv.ddwords[1] = ctx->salt.ddwords[1] ^ cseq ^ session;

    return 0;
}

int encrypt(struct ctx_data *ctx, unsigned char *plaintext, int plaintext_len, unsigned char *ciphertext)
{
    int len;

    int ciphertext_len;

    /* Initialise the encryption operation. IMPORTANT - ensure you use a key
     * and IV size appropriate for your cipher
     * In this example we are using 256 bit AES (i.e. a 256 bit key). The
     * IV size for *most* modes is the same as the block size. For AES this
     * is 128 bits */
    if(1 != EVP_EncryptInit_ex(ctx->ctx, EVP_aes_256_ctr() , NULL, ctx->key.bytes, ctx->iv.bytes))
        handleErrors();

    /* Provide the message to be encrypted, and obtain the encrypted output.
     * EVP_EncryptUpdate can be called multiple times if necessary
     */
    if(1 != EVP_EncryptUpdate(ctx->ctx, ciphertext, &len, plaintext, plaintext_len))
        handleErrors();
    ciphertext_len = len;

    /* Finalise the encryption. Further ciphertext bytes may be written at
     * this stage.
     */
    if(1 != EVP_EncryptFinal_ex(ctx->ctx, ciphertext + len, &len)) handleErrors();
    ciphertext_len += len;

    return ciphertext_len;
}

int decrypt(struct ctx_data *ctx, unsigned char *ciphertext, int ciphertext_len, unsigned char *plaintext)
{
    int len;

    int plaintext_len;

    /* Initialise the decryption operation. IMPORTANT - ensure you use a key
     * and IV size appropriate for your cipher
     * In this example we are using 256 bit AES (i.e. a 256 bit key). The
     * IV size for *most* modes is the same as the block size. For AES this
     * is 128 bits */
    if(1 != EVP_DecryptInit_ex(ctx->ctx, EVP_aes_256_ctr(), NULL, ctx->key.bytes, ctx->iv.bytes))
        handleErrors();

    /* Provide the message to be decrypted, and obtain the plaintext output.
     * EVP_DecryptUpdate can be called multiple times if necessary
     */
    if(1 != EVP_DecryptUpdate(ctx->ctx, plaintext, &len, ciphertext, ciphertext_len))
        handleErrors();
    plaintext_len = len;

    /* Finalise the decryption. Further plaintext bytes may be written at
     * this stage.
     */
    if(1 != EVP_DecryptFinal_ex(ctx->ctx, plaintext + len, &len)) handleErrors();
    plaintext_len += len;

    return plaintext_len;
}


int main (void)
{
    struct ctx_data ctx;

    struct ctx_data ctxd;
    /* A 256 bit key */
    unsigned char *key = (unsigned char *)"01234567890123456789012345678901";

    /* A 128 bit IV */
    unsigned char *iv1 = (unsigned char *)"01234567890123456";
    unsigned char *iv2 = (unsigned char *)"98765432109876543";

    /* Message to be encrypted */
    unsigned char *plaintext =
        (unsigned char *)"The quick brown fox jumps over the lazy dog, The quick brown foxjumps over the lazy do";

    unsigned char ciphertext[128];
    unsigned char decryptedtext[128];

    int decryptedtext_len, ciphertext_len;

    /* Initialise the library */
    ERR_load_crypto_strings();
    OpenSSL_add_all_algorithms();
    OPENSSL_config(NULL);

    init_ctx_data(&ctx, key, strlen(key), NULL, 0);

    init_ctx_data(&ctxd, key, strlen(key), NULL, 0);

    printf ("IV length: %d\n", EVP_CIPHER_iv_length(EVP_aes_256_ctr()));
    update_iv(&ctx, 1, 1);
    /* Encrypt the plaintext */
    ciphertext_len = encrypt (&ctx, plaintext, strlen ((char *)plaintext), ciphertext);

    /* Do something useful with the ciphertext here */
    printf("Ciphertext (%d bytes) is:\n", ciphertext_len);
    BIO_dump_fp (stdout, (const char *)ciphertext, ciphertext_len);

    /* repeat again, use new iv */
    update_iv(&ctx, 1, 2);
    ciphertext_len = encrypt (&ctx, plaintext, strlen ((char *)plaintext), ciphertext);
    printf("Ciphertext (%d bytes) is:\n", ciphertext_len);
    BIO_dump_fp (stdout, (const char *)ciphertext, ciphertext_len);

    /* Decrypt the ciphertext */
    update_iv(&ctxd, 1, 2);
    decryptedtext_len = decrypt(&ctxd, ciphertext, ciphertext_len, decryptedtext);

    /* Clean up */
    EVP_CIPHER_CTX_free(ctx.ctx);
    EVP_CIPHER_CTX_free(ctxd.ctx);

    decryptedtext[decryptedtext_len] = '\0';

    printf("Decrypted text (%d bytes) is:\n", decryptedtext_len);
    printf("%s\n", decryptedtext);

    /* Clean up */
    EVP_cleanup();
    ERR_free_strings();

    return 0;
}
