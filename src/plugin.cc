// indilib_obs.cpp
#include <obs-module.h>
#include <baseclient.h>
#include <indidevapi.h>
#include <mutex>
#include <vector>
#include <cstring>

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE("indilib_obs", "en-US")

struct IndiObsSource {
    obs_source_t *source;
    std::mutex frame_mutex;
    std::vector<uint8_t> rgb_frame;
    uint32_t width;
    uint32_t height;
    bool has_new_frame = false;
};


void newBLOB(IndiObsSource *s)
{
    if (!s)
        return;

        int width = 640;
        int height = 480;

        std::lock_guard<std::mutex> lock(s->frame_mutex);

        s->width = width;
        s->height = height;
        s->rgb_frame.resize(width * height * 4);

        memset(s->rgb_frame.data(), 255, width * height * 4);

        s->has_new_frame = true;
}

gs_texture_t *tex;

// --- OBS Callbacks ---
static void *indilib_source_create(obs_data_t *settings, obs_source_t *source) {
    IndiObsSource *s = new IndiObsSource();
    s->source = source;

    newBLOB(s);

    const uint8_t *planes[1] = { s->rgb_frame.data() };
    tex = gs_texture_create(
        640, 480, GS_RGBA, 1, planes, GS_DYNAMIC
    );

    return s;
}

static void indilib_source_destroy(void *data) {
    IndiObsSource *s = reinterpret_cast<IndiObsSource*>(data);
    gs_texture_destroy(tex);
    delete s;
}

static uint32_t indilib_source_get_width(void *data) {
    return 640;
}

static uint32_t indilib_source_get_height(void *data) {
    return 480;
}

static void indilib_source_video_tick(void *data, float seconds) {
    IndiObsSource *s = reinterpret_cast<IndiObsSource*>(data);
}

static void indilib_source_video_render(void *data, gs_effect_t *effect) {
    IndiObsSource *s = reinterpret_cast<IndiObsSource*>(data);

    newBLOB(s);

    std::lock_guard<std::mutex> lock(s->frame_mutex);
    if (!s->has_new_frame || s->rgb_frame.empty())
        return;

    gs_texture_set_image(tex, s->rgb_frame.data(), 640 * 480 * 4, false);

    gs_blend_state_push();
    gs_blend_function(GS_BLEND_ONE, GS_BLEND_ZERO);
    gs_draw_sprite(tex, 0, 640, 480);
    gs_blend_state_pop();


    s->has_new_frame = false;
}

// --- Регистрация source ---
static struct obs_source_info indilib_source_info = {};
extern "C" bool obs_module_load(void) {
    indilib_source_info.id = "indilib_source";
    indilib_source_info.type = OBS_SOURCE_TYPE_INPUT;
    indilib_source_info.output_flags = OBS_SOURCE_VIDEO;
    indilib_source_info.get_name = [] (void*) -> const char* { return "INDI Camera"; };
    indilib_source_info.create = indilib_source_create;
    indilib_source_info.destroy = indilib_source_destroy;
    indilib_source_info.get_width = indilib_source_get_width;
    indilib_source_info.get_height = indilib_source_get_height;
    indilib_source_info.video_tick = indilib_source_video_tick;
    indilib_source_info.video_render = indilib_source_video_render;

    obs_register_source(&indilib_source_info);
    return true;
}
