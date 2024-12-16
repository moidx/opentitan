

otcrypto_status_t otcrypto_ed25519_keygen(
    otcrypto_blinded_key_t *private_key, otcrypto_unblinded_key_t *public_key) {
  // TODO: Ed25519 is not yet implemented.
  return OTCRYPTO_NOT_IMPLEMENTED;
}

otcrypto_status_t otcrypto_ed25519_sign(
    const otcrypto_blinded_key_t *private_key,
    otcrypto_const_byte_buf_t input_message,
    otcrypto_eddsa_sign_mode_t sign_mode, otcrypto_word32_buf_t signature) {
  // TODO: Ed25519 is not yet implemented.
  return OTCRYPTO_NOT_IMPLEMENTED;
}

otcrypto_status_t otcrypto_ed25519_verify(
    const otcrypto_unblinded_key_t *public_key,
    otcrypto_const_byte_buf_t input_message,
    otcrypto_eddsa_sign_mode_t sign_mode, otcrypto_const_word32_buf_t signature,
    hardened_bool_t *verification_result) {
  // TODO: Ed25519 is not yet implemented.
  return OTCRYPTO_NOT_IMPLEMENTED;
}

otcrypto_status_t otcrypto_ed25519_keygen_async_start(
    const otcrypto_blinded_key_t *private_key) {
  // TODO: Ed25519 is not yet implemented.
  return OTCRYPTO_NOT_IMPLEMENTED;
}

otcrypto_status_t otcrypto_ed25519_keygen_async_finalize(
    otcrypto_blinded_key_t *private_key, otcrypto_unblinded_key_t *public_key) {
  // TODO: Ed25519 is not yet implemented.
  return OTCRYPTO_NOT_IMPLEMENTED;
}

otcrypto_status_t otcrypto_ed25519_sign_async_start(
    const otcrypto_blinded_key_t *private_key,
    otcrypto_const_byte_buf_t input_message,
    otcrypto_eddsa_sign_mode_t sign_mode, otcrypto_word32_buf_t signature) {
  // TODO: Ed25519 is not yet implemented.
  return OTCRYPTO_NOT_IMPLEMENTED;
}

otcrypto_status_t otcrypto_ed25519_sign_async_finalize(
    otcrypto_word32_buf_t signature) {
  // TODO: Ed25519 is not yet implemented.
  return OTCRYPTO_NOT_IMPLEMENTED;
}

otcrypto_status_t otcrypto_ed25519_verify_async_start(
    const otcrypto_unblinded_key_t *public_key,
    otcrypto_const_byte_buf_t input_message,
    otcrypto_eddsa_sign_mode_t sign_mode,
    otcrypto_const_word32_buf_t signature) {
  // TODO: Ed25519 is not yet implemented.
  return OTCRYPTO_NOT_IMPLEMENTED;
}

otcrypto_status_t otcrypto_ed25519_verify_async_finalize(
    hardened_bool_t *verification_result) {
  // TODO: Ed25519 is not yet implemented.
  return OTCRYPTO_NOT_IMPLEMENTED;
}
