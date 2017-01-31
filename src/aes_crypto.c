#include "aes_crypto.h"

static
void handleErrors(void)
{
    ERR_print_errors_fp(stderr);
    abort();
}

void init_crypto(void)
{
    /* Initialise the library */
    ERR_load_crypto_strings();
    OpenSSL_add_all_algorithms();
    OPENSSL_config(NULL);
}

void destroy_crypto(void)
{
    /* Clean up */
    EVP_cleanup();
    ERR_free_strings();
}

int init_ctx_data(struct ctx_data* ctx, unsigned char *key, size_t key_len, unsigned char *salt, size_t salt_len)
{
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

    return 0;
}

void destroy_ctx_data(struct ctx_data* ctx)
{
    /* Clean up */
    EVP_CIPHER_CTX_free(ctx->ctx);
}

int update_iv(struct ctx_data* ctx, uint64_t session, uint64_t cseq)
{
    //IV = (k_s * 2^16) XOR (SSRC * 2^64) XOR (i * 2^16)
    ctx->iv.ddwords[0] = ctx->salt.ddwords[0] ^ cseq ^ session;
    ctx->iv.ddwords[1] = ctx->salt.ddwords[1] ^ cseq ^ session;

    return 0;
}

int aes_encrypt(struct ctx_data *ctx, unsigned char *plaintext, int plaintext_len, unsigned char *ciphertext)
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

int aes_decrypt(struct ctx_data *ctx, unsigned char *ciphertext, int ciphertext_len, unsigned char *plaintext)
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
