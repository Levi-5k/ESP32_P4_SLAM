# Install script for directory: W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/VisualSLAM_INAV")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "W:/Espressif/tools/riscv32-esp-elf/esp-14.2.0_20241119/riscv32-esp-elf/bin/riscv32-esp-elf-objdump.exe")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mbedtls" TYPE FILE PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ FILES
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/aes.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/aria.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/asn1.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/asn1write.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/base64.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/bignum.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/block_cipher.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/build_info.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/camellia.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/ccm.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/chacha20.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/chachapoly.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/check_config.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/cipher.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/cmac.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/compat-2.x.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/config_adjust_legacy_crypto.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/config_adjust_legacy_from_psa.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/config_adjust_psa_from_legacy.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/config_adjust_psa_superset_legacy.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/config_adjust_ssl.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/config_adjust_x509.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/config_psa.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/constant_time.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/ctr_drbg.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/debug.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/des.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/dhm.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/ecdh.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/ecdsa.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/ecjpake.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/ecp.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/entropy.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/error.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/gcm.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/hkdf.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/hmac_drbg.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/lms.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/mbedtls_config.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/md.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/md5.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/memory_buffer_alloc.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/net_sockets.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/nist_kw.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/oid.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/pem.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/pk.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/pkcs12.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/pkcs5.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/pkcs7.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/platform.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/platform_time.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/platform_util.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/poly1305.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/private_access.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/psa_util.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/ripemd160.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/rsa.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/sha1.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/sha256.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/sha3.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/sha512.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/ssl.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/ssl_cache.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/ssl_ciphersuites.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/ssl_cookie.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/ssl_ticket.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/threading.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/timing.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/version.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/x509.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/x509_crl.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/x509_crt.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/mbedtls/x509_csr.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/psa" TYPE FILE PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ FILES
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/build_info.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_adjust_auto_enabled.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_adjust_config_dependencies.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_adjust_config_key_pair_types.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_adjust_config_synonyms.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_builtin_composites.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_builtin_key_derivation.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_builtin_primitives.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_compat.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_config.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_driver_common.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_driver_contexts_composites.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_driver_contexts_key_derivation.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_driver_contexts_primitives.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_extra.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_legacy.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_platform.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_se_driver.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_sizes.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_struct.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_types.h"
    "W:/Espressif/frameworks/esp-idf-v5.5/components/mbedtls/mbedtls/include/psa/crypto_values.h"
    )
endif()

