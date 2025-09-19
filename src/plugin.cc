// indilib_obs.cpp
#include <obs-module.h>
#include <cstring>
#include <mutex>
#include <vector>
#include <list>
#include <map>
#include <cstring>
#include <memory>
#include <iostream>
#include <algorithm>
#include <unistd.h>


#include "indi_client.hh"

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE("indilib_obs", "en-US")

class ObsIndiClient;

struct IndiObsSource
{
    std::string indi_server_addr;
    int indi_server_port;
    bool connected;

    obs_source_t *source;

    std::mutex frame_mutex;
    bool is_streaming = false;
    std::unique_ptr<ObsIndiClient> indiClient;
};


// --- OBS Callbacks ---
static void *indilib_source_create(obs_data_t *settings, obs_source_t *source)
{
    IndiObsSource *s = new IndiObsSource();
    s->indiClient = std::make_unique<ObsIndiClient>(s->frame_mutex);
    s->source = source;

    s->indiClient->processBLOB(nullptr);

    s->indi_server_addr = obs_data_get_string(settings, "server");
    s->indi_server_port   = (int)obs_data_get_int(settings, "port");
    s->indiClient->selectCCD(obs_data_get_string(settings, "ccd"));

    s->indiClient->setServer(s->indi_server_addr.c_str(), s->indi_server_port);
    if (!s->indiClient->connectServer()) {
        blog(LOG_INFO, "Can't connect to server: %s:%d", s->indi_server_addr.c_str(), s->indi_server_port);
        s->connected = false;
    } else {
        s->connected = true;
    }
    return s;
}

static void indilib_source_destroy(void *data)
{
    IndiObsSource *s = reinterpret_cast<IndiObsSource *>(data);

    s->indiClient->disconnectServer();
    delete s;
}

static uint32_t indilib_source_get_width(void *data)
{
    IndiObsSource *s = reinterpret_cast<IndiObsSource *>(data);
    //blog(LOG_INFO, "Get width: %i", s->width);
    return s->indiClient->indiFrameWidth();
}

static uint32_t indilib_source_get_height(void *data)
{
    IndiObsSource *s = reinterpret_cast<IndiObsSource *>(data);
    //blog(LOG_INFO, "Get height: %i", s->height);
    return s->indiClient->indiFrameHeight();
}

static void indilib_source_video_tick(void *data, float seconds)
{
    IndiObsSource *s = reinterpret_cast<IndiObsSource *>(data);
}

static void indilib_source_video_render(void *data, gs_effect_t *effect)
{
    IndiObsSource *s = reinterpret_cast<IndiObsSource *>(data);

    std::lock_guard<std::mutex> lock(s->frame_mutex);
    if (s->indiClient->indiFrame().empty())
        return;

    obs_enter_graphics();
    const uint8_t *planes[1] = {s->indiClient->indiFrame().data()};
    gs_texture_t *tex = gs_texture_create(s->indiClient->indiFrameWidth(), s->indiClient->indiFrameHeight(),
                                          GS_RGBA, 1, planes, GS_DYNAMIC);
    obs_leave_graphics();

    const bool previous = gs_framebuffer_srgb_enabled();
    gs_enable_framebuffer_srgb(true);

    gs_blend_state_push();
    gs_blend_function(GS_BLEND_ONE, GS_BLEND_INVSRCALPHA);

    gs_eparam_t *const param = gs_effect_get_param_by_name(effect, "image");
    gs_effect_set_texture_srgb(param, tex);

    gs_draw_sprite(tex, 0, s->indiClient->indiFrameWidth(), s->indiClient->indiFrameHeight());
    gs_blend_state_pop();
    gs_enable_framebuffer_srgb(previous);

    obs_enter_graphics();
    gs_texture_destroy(tex);
    obs_leave_graphics();
}

static void server_modified(IndiObsSource *s, const std::string &server, int port)
{
    if (s->connected) {
        s->indiClient->disconnectServer();
    }
    s->indi_server_addr = server;
    s->indi_server_port = port;
    s->indiClient->setServer(s->indi_server_addr.c_str(), s->indi_server_port);
    if (!s->indiClient->connectServer()) {
        blog(LOG_INFO, "Can't connect to server: %s:%d", s->indi_server_addr.c_str(), s->indi_server_port);
        s->connected = false;
    } else {
        s->connected = true;
    }
}

static obs_properties_t *indilib_source_get_properties(void *data)
{
    IndiObsSource *s = reinterpret_cast<IndiObsSource *>(data);
    obs_properties_t *props = obs_properties_create();

    // INDI server
    obs_property_t *server_ip = obs_properties_add_text(props, "server", "INDI Server Host", OBS_TEXT_DEFAULT);
    obs_property_set_modified_callback2(server_ip,
        [](void *data, obs_properties_t *props, obs_property_t *property, obs_data_t *settings) -> bool {
            IndiObsSource *s = reinterpret_cast<IndiObsSource *>(data);

            std::string server = obs_data_get_string(settings, "server");
            if (server != s->indi_server_addr) {
                server_modified(s, server, s->indi_server_port);
                blog(LOG_INFO, "INDI server ip changed");
                obs_property_t *ccd_list = obs_properties_get(props, "ccd");
                obs_property_list_clear(ccd_list);
                if (s->connected) {
                    usleep(100000);
                    for (auto ccd : s->indiClient->indiCCDs()) {
                        auto ccddev = s->indiClient->ccddev_encode(ccd);
                        obs_property_list_add_string(ccd_list, ccddev.c_str(), ccddev.c_str());
                    }
                }
            }

            return true;
        }, s);

    // Add a number field (port)
    obs_property_t *server_port = obs_properties_add_int(props, "port", "INDI Server Port", 1, 65535, 1);
    obs_property_set_modified_callback2(server_port,
        [](void *data, obs_properties_t *props, obs_property_t *property, obs_data_t *settings) -> bool {
            IndiObsSource *s = reinterpret_cast<IndiObsSource *>(data);

            int port = (int)obs_data_get_int(settings, "port");
            if (port != s->indi_server_port) {
                server_modified(s, s->indi_server_addr, port);
                blog(LOG_INFO, "INDI server port updated");
                obs_property_t *ccd_list = obs_properties_get(props, "ccd");
                obs_property_list_clear(ccd_list);
                if (s->connected) {
                    usleep(100000);
                    for (auto ccd : s->indiClient->indiCCDs()) {
                        auto ccddev = s->indiClient->ccddev_encode(ccd);
                        obs_property_list_add_string(ccd_list, ccddev.c_str(), ccddev.c_str());
                    }
                }
            }

            return true;
        }, s);

    // Add a dropdown (list of CCDs)
    obs_property_t *list = obs_properties_add_list(
        props,
        "ccd",
        "CCD Device",
        OBS_COMBO_TYPE_LIST,
        OBS_COMBO_FORMAT_STRING);

    obs_property_set_modified_callback2(list,
        [](void *data, obs_properties_t *props, obs_property_t *property, obs_data_t *settings) -> bool {
            IndiObsSource *s = reinterpret_cast<IndiObsSource *>(data);

            std::string ccddev = obs_data_get_string(settings, "ccd");
            auto ccd = s->indiClient->ccddev_decode(ccddev);
            if (ccd != s->indiClient->indiCurrentCCD()) {
                s->indiClient->selectCCD(ccd);
            }
            return true;
        }, s);

    for (auto ccd : s->indiClient->indiCCDs()) {
        auto ccddev = s->indiClient->ccddev_encode(ccd);
        obs_property_list_add_string(list, ccddev.c_str(), ccddev.c_str());
    }

    return props;
}


static void indilib_source_defaults(obs_data_t *settings)
{
    // Set a default hostname
    obs_data_set_default_string(settings, "server", "localhost");

    // Default port for INDI server
    obs_data_set_default_int(settings, "port", 7624);

    // Default CCD selection (empty until devices are queried)
    obs_data_set_default_string(settings, "ccd", ":");
}


// --- Регистрация source ---
static struct obs_source_info indilib_source_info = {};
extern "C" bool obs_module_load(void)
{
    indilib_source_info.id = "indilib_source";
    indilib_source_info.type = OBS_SOURCE_TYPE_INPUT;
    indilib_source_info.output_flags = OBS_SOURCE_VIDEO;
    indilib_source_info.get_name = [](void *) -> const char *
    { return "INDI Camera"; };
    indilib_source_info.create = indilib_source_create;
    indilib_source_info.destroy = indilib_source_destroy;
    indilib_source_info.get_width = indilib_source_get_width;
    indilib_source_info.get_height = indilib_source_get_height;
    indilib_source_info.video_tick = indilib_source_video_tick;
    indilib_source_info.video_render = indilib_source_video_render;
    indilib_source_info.get_properties = indilib_source_get_properties;
    indilib_source_info.get_defaults = indilib_source_defaults;

    obs_register_source(&indilib_source_info);
    return true;
}
