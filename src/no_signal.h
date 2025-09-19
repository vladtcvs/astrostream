#pragma once

#ifdef __cplusplus
extern "C" {
#endif

struct image_array_s {
    unsigned int 	 width;
    unsigned int 	 height;
    unsigned char	 pixel_data[320 * 240 * 4 + 1];
};

extern const struct image_array_s no_signal_image;

#ifdef __cplusplus
}
#endif
