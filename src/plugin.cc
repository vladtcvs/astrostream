// indilib_obs.cpp
#include <obs-module.h>
#include <baseclient.h>
#include <basedevice.h>
#include <indidevapi.h>
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
#include <fitsio.h>

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE("indilib_obs", "en-US")

static std::pair<std::string, std::string> ccddev_decode(const std::string& ccddev);
static std::string ccddev_encode(const std::pair<std::string, std::string> &ccd);

class ObsIndiClient;

struct IndiObsSource
{
    std::string indi_server_addr;
    int indi_server_port;
    std::pair<std::string, std::string> ccd;
    std::list<std::pair<std::string, std::string>> ccds;
    bool connected;

    obs_source_t *source;

    std::mutex frame_mutex;
    std::vector<uint8_t> rgb_frame;
    uint32_t width;
    uint32_t height;
    bool is_streaming = false;
    bool has_new_frame = false;
    std::unique_ptr<ObsIndiClient> indiClient;
};

class ObsIndiClient : public INDI::BaseClient
{
public:
    IndiObsSource *obs_source;
    std::list<std::string> devices;
    ObsIndiClient(IndiObsSource *source) : obs_source(source) {}

    void startStream()
    {
        if (obs_source->is_streaming)
            stopStream();
        std::cout << "Starting stream from " << ccddev_encode(obs_source->ccd) << std::endl;
        obs_source->is_streaming = true;
    }

    void stopStream()
    {
        if (!obs_source->is_streaming)
            return;
        std::cout << "Stop stream from " << ccddev_encode(obs_source->ccd) << std::endl;
        obs_source->is_streaming = false;
    }

    int indiFrameWidth()
    {
        return obs_source->width;
    }

    int indiFrameHeight()
    {
        return obs_source->height;
    }

    uint8_t getPixel(const uint8_t *data, size_t w, size_t h, int bitpix, int x, int y, int plane)
    {
        int bpp = 1;
        if (bitpix == 16)
            bpp = 2;
        else if (bitpix == 32)
            bpp = 4;

        uint32_t val = 0;
        uint32_t bytes[4] = {0};
        
        bytes[0] = data[(plane*(w*h) + y*w + x)*bpp];
        if (bpp >= 2)
            bytes[1] = data[(plane*(w*h) + y*w + x)*bpp+1];
        if (bpp >= 4) {
            bytes[2] = data[(plane*(w*h) + y*w + x)*bpp+2];
            bytes[3] = data[(plane*(w*h) + y*w + x)*bpp+3];
        }
        val = bytes[3] << 24 | bytes[2] << 16 | bytes[1] << 8 | bytes[0];
        if (bitpix > 8)
            val >>= (bitpix - 8);
        else if (bitpix < 8)
            val <<= (8 - bitpix);
        return val & 0xFFU;
    }

    void indiFillFrame(IBLOB *indiBlob)
    {
        if (!strcmp(indiBlob->format, ".fits"))
        {
            fitsfile *fptr;
            int status = 0;
            size_t size = indiBlob->size;
            if (fits_open_memfile(&fptr, "mem://blob", READONLY, &indiBlob->blob, &size, 0, nullptr, &status) != 0) {
                blog(LOG_INFO, "Can not read fits, skipping");
                return;
            }

            int naxis;
            long naxes[3];
            int bitpix;
            if (fits_get_img_param(fptr, 3, &bitpix, &naxis, naxes, &status)) {
                std::cerr << "Error getting image parameters" << std::endl;
                fits_report_error(stderr, status);
                fits_close_file(fptr, &status);
                return;
            }

            if (!(naxis == 2 || (naxis == 3 && (naxes[2] == 3)))) {
                std::cerr << "Expected 2D or 3D RGB image, got naxis = " << naxis << std::endl;
                fits_close_file(fptr, &status);
                return;
            }

            int bpp = 1;
            if (bitpix == 16)
                bpp = 2;
            else if (bitpix == 32)
                bpp = 4;

            // Allocate buffer for pixel data
            obs_source->width = naxes[0];
            obs_source->height = naxes[1];
            obs_source->rgb_frame.resize(obs_source->width * obs_source->height * 4);

            long nbytes = naxes[0] * naxes[1] * bpp;
            if (naxis == 3)
                nbytes *= naxes[2];

            std::vector<uint8_t> pixels(nbytes);

            std::cout << "Read data" << std::endl;

            // Read image data
            if (fits_read_img(fptr, TBYTE, 1, nbytes, nullptr, pixels.data(), nullptr, &status)) {
                std::cerr << "Error reading image data" << std::endl;
                fits_report_error(stderr, status);
                fits_close_file(fptr, &status);
                return;
            }

            // Close FITS file
            fits_close_file(fptr, &status);

            std::cout << "Copy data" << std::endl;
            for (int y = 0; y < obs_source->height; y++)
            {
                std::cout << y << std::endl;
                for (int x = 0; x < obs_source->width; x++)
                {
                    int w = obs_source->width;
                    int h = obs_source->height;
                    obs_source->rgb_frame.data()[4*(y*w + x) + 0] = getPixel(pixels.data(), w, h, bitpix, x, y, 0);
                    if (naxis == 3)
                    {
                        obs_source->rgb_frame.data()[4*(y*w + x) + 1] = getPixel(pixels.data(), w, h, bitpix, x, y, 1);
                        obs_source->rgb_frame.data()[4*(y*w + x) + 2] = getPixel(pixels.data(), w, h, bitpix, x, y, 2);
                    } else {
                        obs_source->rgb_frame.data()[4*(y*w + x) + 1] = obs_source->rgb_frame.data()[4*(y*w + x) + 0];
                        obs_source->rgb_frame.data()[4*(y*w + x) + 2] = obs_source->rgb_frame.data()[4*(y*w + x) + 0];
                    }
                    obs_source->rgb_frame.data()[4*(y*w + x) + 3] = 255;
                }
            }

            blog(LOG_INFO, "Fill frame %ix%i", obs_source->width, obs_source->height);
            //memset(obs_source->rgb_frame.data(), 155, obs_source->width * obs_source->height * 4);
        } else {
            blog(LOG_INFO, "Unknown blob format %s, skipping", indiBlob->format);
            return;
        }
    }

    void dummyFillFrame(size_t new_w, size_t new_h)
    {
        blog(LOG_INFO, "Fill dummy frame %ix%i", obs_source->width, obs_source->height);
        obs_source->width = new_w;
        obs_source->height = new_h;
        obs_source->rgb_frame.resize(obs_source->width * obs_source->height * 4);
        memset(obs_source->rgb_frame.data(), 255, obs_source->width * obs_source->height * 4);
    }

    void newDevice(INDI::BaseDevice baseDevice) override
    {
        std::string name(baseDevice.getDeviceName());
        devices.push_back(name);
        std::cout << "New device: " << name << std::endl;
    }

    void removeDevice(INDI::BaseDevice baseDevice) override
    {
        std::string name(baseDevice.getDeviceName());
        auto it = std::find(devices.begin(), devices.end(), name);
        if (it != devices.end())
            devices.erase(it);
        std::cout << "Remove device: " << name << std::endl;
    }

    void newProperty(INDI::Property property) override
    {
        std::string device_name = property.getDeviceName();    
        std::string property_name = property.getName();
        auto ptype = property.getType();
        std::cout << "New property: " << property_name << " for device " << device_name << " (" << ptype << ")"<< std::endl;
        if (ptype != INDI_BLOB)
            return;

        INDI::BaseDevice dev = getDevice(device_name.c_str());
        if (!(dev.getDriverInterface() & INDI::BaseDevice::CCD_INTERFACE))
            return;

        auto ccd = std::make_pair(property_name, device_name);
        obs_source->ccds.push_back(ccd);
        if (ccd == obs_source->ccd) {
            startStream();
            setBLOBMode(B_ALSO, device_name.c_str(), property_name.c_str());
        }
    }

    void updateProperty(INDI::Property property) override
    {
        // Called when a property is updated
        std::string property_name = property.getName();
        std::string device_name = property.getDeviceName();

        auto prop = std::make_pair(property_name, device_name);

        if (std::find(obs_source->ccds.begin(), obs_source->ccds.end(), prop) == obs_source->ccds.end())
            return;

        std::cout << "CCD updated: " << property.getName() << " for device " << property.getDeviceName() << std::endl;
        auto blobProp = property.getBLOB();
        for (auto blob : *blobProp)
        {
            processBLOB(&blob);
        }
    }

    void removeProperty(INDI::Property property) override
    {
        std::string device_name = property.getDeviceName();    

        // Called when a property is removed
        std::string property_name = property.getName();

        std::cout << "Property removed: " << property_name << " for device " << device_name << std::endl;
        auto ccd = std::make_pair(property_name, device_name);
        auto it = std::find(obs_source->ccds.begin(), obs_source->ccds.end(), ccd);
        if (it != obs_source->ccds.end()) {
            if (ccd == obs_source->ccd) {
                stopStream();
            }
            obs_source->ccds.erase(it);
        }
    }

    void newMessage(INDI::BaseDevice dp, int messageID) override
    {
        // Called when a new message is received from a device
        std::cout << "New message from " << dp.getDeviceName() << ": " << dp.messageQueue(messageID) << std::endl;
    }

    void processBLOB(IBLOB *indiBlob)
    {
        if (!obs_source)
            return;

        std::lock_guard<std::mutex> lock(obs_source->frame_mutex);

        if (indiBlob == nullptr) {
            // Display dummy image
            // TODO: add "no signal" text
            int new_w = 640;
            int new_h = 480;

            if (new_w != obs_source->width || new_h != obs_source->height) {
                blog(LOG_INFO, "Image size changed %ix%i, old was %ix%i", new_w, new_h, obs_source->width, obs_source->height);
            }

            dummyFillFrame(new_w, new_h);
            obs_source->has_new_frame = true;
        } else {
            indiFillFrame(indiBlob);
        }
    }
};

// --- OBS Callbacks ---
static void *indilib_source_create(obs_data_t *settings, obs_source_t *source)
{
    IndiObsSource *s = new IndiObsSource();
    s->indiClient = std::make_unique<ObsIndiClient>(s);
    s->source = source;

    s->indiClient->processBLOB(nullptr);

    s->indi_server_addr = obs_data_get_string(settings, "server");
    s->indi_server_port   = (int)obs_data_get_int(settings, "port");
    s->ccd = ccddev_decode(std::string(obs_data_get_string(settings, "ccd")));

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
    return s->width;
}

static uint32_t indilib_source_get_height(void *data)
{
    IndiObsSource *s = reinterpret_cast<IndiObsSource *>(data);
    //blog(LOG_INFO, "Get height: %i", s->height);
    return s->height;
}

static void indilib_source_video_tick(void *data, float seconds)
{
    IndiObsSource *s = reinterpret_cast<IndiObsSource *>(data);
}

static void indilib_source_video_render(void *data, gs_effect_t *effect)
{
    IndiObsSource *s = reinterpret_cast<IndiObsSource *>(data);

    std::lock_guard<std::mutex> lock(s->frame_mutex);
    if (s->rgb_frame.empty())
        return;

    obs_enter_graphics();
    const uint8_t *planes[1] = {s->rgb_frame.data()};
    gs_texture_t *tex = gs_texture_create(s->width, s->height, GS_RGBA, 1, planes, GS_DYNAMIC);
    obs_leave_graphics();

    const bool previous = gs_framebuffer_srgb_enabled();
    gs_enable_framebuffer_srgb(true);

    gs_blend_state_push();
    gs_blend_function(GS_BLEND_ONE, GS_BLEND_INVSRCALPHA);

    gs_eparam_t *const param = gs_effect_get_param_by_name(effect, "image");
    gs_effect_set_texture_srgb(param, tex);

    gs_draw_sprite(tex, 0, s->width, s->height);
    gs_blend_state_pop();
    gs_enable_framebuffer_srgb(previous);

    obs_enter_graphics();
    gs_texture_destroy(tex);
    obs_leave_graphics();

    s->has_new_frame = false;
}

static void server_modified(IndiObsSource *s, const std::string &server, int port)
{
    if (s->connected) {
        s->indiClient->stopStream();
        s->indiClient->disconnectServer();
    }
    s->indi_server_addr = server;
    s->indi_server_port = port;
    s->ccds.clear();
    s->indiClient->setServer(s->indi_server_addr.c_str(), s->indi_server_port);
    if (!s->indiClient->connectServer()) {
        blog(LOG_INFO, "Can't connect to server: %s:%d", s->indi_server_addr.c_str(), s->indi_server_port);
        s->connected = false;
    } else {
        s->connected = true;
    }
}

static void ccd_modified(IndiObsSource *s, const std::pair<std::string, std::string>& ccd)
{
    if (s->connected)
        s->indiClient->stopStream();
    s->ccd = ccd;
    if (s->connected)
        s->indiClient->startStream();
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
                    for (auto ccd : s->ccds) {
                        auto ccddev = ccddev_encode(ccd);
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
                    for (auto ccd : s->ccds) {
                        auto ccddev = ccddev_encode(ccd);
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
            std::pair<std::string, std::string> ccd = ccddev_decode(ccddev);
            if (ccd != s->ccd) {
                ccd_modified(s, ccd);
            }
            return true;
        }, s);

    for (auto ccd : s->ccds) {
        auto ccddev = ccddev_encode(ccd);
        obs_property_list_add_string(list, ccddev.c_str(), ccddev.c_str());
    }

    return props;
}

static std::pair<std::string, std::string> ccddev_decode(const std::string& ccddev)
{
    size_t pos = ccddev.find(":");
    std::pair<std::string, std::string> newccd;
    if (pos == ccddev.npos)
        newccd    = std::make_pair("", "");
    else
        newccd    = std::make_pair(ccddev.substr(0, pos), ccddev.substr(pos + 1));
    return newccd;
}

static std::string ccddev_encode(const std::pair<std::string, std::string> &ccd)
{
    return ccd.first + ":" + ccd.second;
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
