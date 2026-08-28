#ifndef PRODUCT_CONFIG_H
#define PRODUCT_CONFIG_H

/* Product identity. PRODUCT_VERSION_NUMBER is sent to the LCD and tuning host. */
#define PRODUCT_MODEL_NAME          "9C-HTP100"
#define PRODUCT_VERSION_NUMBER      20260828UL

/* 1U: consume/fuse an eye shield at the first formal treatment actuation.
 * 0U: do not write or enforce the eye-shield consumed marker. */
#ifndef PRODUCT_EYE_FUSE_ENABLED
#define PRODUCT_EYE_FUSE_ENABLED    0U
#endif

#if PRODUCT_EYE_FUSE_ENABLED != 0U && PRODUCT_EYE_FUSE_ENABLED != 1U
#error "PRODUCT_EYE_FUSE_ENABLED must be 0U or 1U"
#endif

#endif
