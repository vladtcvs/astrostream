/*
 * Copyright (c) 2025 Vladislav Tsendrovskii
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, version 2.1 of the License.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "indi_client.hh"
#include <fitsio.h>
#include <jpeglib.h>
#include <iostream>
#include <obs-module.h>

#include "no_signal.h"

int ObsIndiClient::indiFrameWidth() const
{
    return width;
}

int ObsIndiClient::indiFrameHeight() const
{
    return height;
}

const std::vector<uint8_t>& ObsIndiClient::indiFrame() const
{
    return rgba_frame;
}

std::list<std::string> ObsIndiClient::indiCCDs() const
{
    std::list<std::string> ccds;
    for (auto it = devices.begin(); it != devices.end(); it++) {
        if (it->second.isCCD)
            ccds.push_back(it->second.device_name);
    }
    return ccds;
}

std::string ObsIndiClient::indiCurrentCCD() const
{
    return current_ccd;
}

bool ObsIndiClient::connectServer()
{
    devices.clear();
    dummyFillFrame();
    return INDI::BaseClient::connectServer();
}

bool ObsIndiClient::disconnectServer(int exit_code)
{
    devices.clear();
    current_ccd = "";
    dummyFillFrame();
    return INDI::BaseClient::disconnectServer(exit_code);
}

void ObsIndiClient::selectCCD(const std::string &ccd)
{
    if (current_ccd == ccd)
        return;

    if (current_ccd != "" && devices.find(current_ccd) != devices.end()) {
        auto property_name = devices[current_ccd].property_name;
        auto device_name = devices[current_ccd].device_name;
        std::cout << "Disable blob data on " << property_name << ":" << device_name << std::endl;
        setBLOBMode(B_NEVER, device_name.c_str(), property_name.c_str());
    }

    current_ccd = ccd;

    if (current_ccd != "" && devices.find(current_ccd) != devices.end()) {
        if (devices[current_ccd].isCCD) {
            auto property_name = devices[current_ccd].property_name;
            auto device_name = devices[current_ccd].device_name;
            std::cout << "Enable blob data on " << property_name << ":" << device_name << std::endl;
            setBLOBMode(B_ALSO, device_name.c_str(), property_name.c_str());
        }
    }
    dummyFillFrame();
}

uint8_t ObsIndiClient::fitsGetPixel(const std::vector<uint8_t> &data, size_t w, size_t h, int bitpix, int x, int y, int plane) const
{
    int bpp = 1;
    if (bitpix > 8)
        bpp = 2;
    if (bitpix > 32)
        bpp = 4;

    uint32_t val = 0;
    uint32_t bytes[4] = {0};

    bytes[0] = data[(plane * (w * h) + y * w + x) * bpp];
    if (bpp >= 2)
        bytes[1] = data[(plane * (w * h) + y * w + x) * bpp + 1];
    if (bpp >= 4)
    {
        bytes[2] = data[(plane * (w * h) + y * w + x) * bpp + 2];
        bytes[3] = data[(plane * (w * h) + y * w + x) * bpp + 3];
    }
    val = bytes[3] << 24 | bytes[2] << 16 | bytes[1] << 8 | bytes[0];
    if (bitpix > 8)
        val >>= (bitpix - 8);
    else if (bitpix < 8)
        val <<= (8 - bitpix);
    return val & 0xFFU;
}

void ObsIndiClient::indiFillFrameJpeg(const uint8_t *data, size_t size)
{
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;

    if (size < 4 || data[0] != 0xFF || data[1] != 0xD8 || data[size-2] != 0xFF || data[size-1] != 0xD9) {
        std::cout << "Not JPEG" << std::endl;
        return;
    }


    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);

    
    // Set up JPEG data source
    jpeg_mem_src(&cinfo, data, size);

    // Read JPEG header
    if (jpeg_read_header(&cinfo, true) != JPEG_HEADER_OK) {
        std::cout << "Not JPEG" << std::endl;
        jpeg_destroy_decompress(&cinfo);
        return;
    }

    // Start decompression (assume 8-bit RGB)
    jpeg_start_decompress(&cinfo);

    int components = cinfo.output_components; // Should be 3 for RGB
    if (components != 3)
    {
        std::cerr << "Expected 3 components (RGB), got " << components << std::endl;
        jpeg_destroy_decompress(&cinfo);
        return;
    }

    // Allocate buffer for pixel data
    width = cinfo.output_width;
    height = cinfo.output_height;
    rgba_frame.resize(width * height * 4);

    // Allocate buffer for one scanline
    std::vector<uint8_t> scanline(width * 3);
    JSAMPROW row_pointer[1] = {scanline.data()};

    // Read pixel data into a full image buffer
    uint8_t *rgba = rgba_frame.data();
    const uint8_t *sl = scanline.data();
    int dstpos = 0;
    while (cinfo.output_scanline < cinfo.output_height)
    {
        jpeg_read_scanlines(&cinfo, row_pointer, 1);
        int srcpos = 0;
        for (int x = 0; x < width; x++)
        {
            rgba[dstpos++] = sl[srcpos++];
            rgba[dstpos++] = sl[srcpos++];
            rgba[dstpos++] = sl[srcpos++];
            rgba[dstpos++] = 255;
        }
    }

    // Finish decompression
    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
}

void ObsIndiClient::indiFillFrameFITS(uint8_t *data, size_t size)
{
    fitsfile *fptr;
    int status = 0;
    if (fits_open_memfile(&fptr, "mem://blob", READONLY, (void**)(&data), &size, 0, nullptr, &status) != 0)
    {
        blog(LOG_INFO, "Can not read fits, skipping");
        fits_report_error(stderr, status);
        return;
    }

    int naxis;
    long naxes[3];
    int bitpix;
    if (fits_get_img_param(fptr, 3, &bitpix, &naxis, naxes, &status))
    {
        std::cerr << "Error getting image parameters" << std::endl;
        fits_report_error(stderr, status);
        fits_close_file(fptr, &status);
        return;
    }

    if (!(naxis == 2 || (naxis == 3 && (naxes[2] == 3))))
    {
        std::cerr << "Expected 2D or 3D RGB image, got naxis = " << naxis << std::endl;
        fits_close_file(fptr, &status);
        return;
    }

    int imgtype = TBYTE;
    int bpp = 1;
    if (bitpix == 16) {
        bpp = 2;
        imgtype = TUSHORT;
    } else if (bitpix == 32) {
        bpp = 4;
        imgtype = TUINT;
    }

    // Allocate buffer for pixel data
    width = naxes[0];
    height = naxes[1];
    rgba_frame.resize(width * height * 4);

    long npixels = naxes[0] * naxes[1];
    if (naxis == 3)
        npixels *= naxes[2];

    std::vector<uint8_t> pixels(npixels * bpp);

    // Read image data
    int img_type;
    if (fits_read_img(fptr, imgtype, 1, npixels, nullptr, pixels.data(), nullptr, &status))
    {
        std::cerr << "Error reading image data" << std::endl;
        fits_report_error(stderr, status);
        fits_close_file(fptr, &status);
        return;
    }

    // Close FITS file
    fits_close_file(fptr, &status);

    int dstpos = 0;
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            rgba_frame[dstpos++] = fitsGetPixel(pixels, width, height, bitpix, x, y, 0);
            if (naxis == 3)
            {
                rgba_frame[dstpos++] = fitsGetPixel(pixels, width, height, bitpix, x, y, 1);
                rgba_frame[dstpos++] = fitsGetPixel(pixels, width, height, bitpix, x, y, 2);
            }
            else
            {
                rgba_frame[dstpos++] = rgba_frame[dstpos - 1];
                rgba_frame[dstpos++] = rgba_frame[dstpos - 1];
            }
            rgba_frame[dstpos++] = 255;
        }
    }
}

void ObsIndiClient::indiFillFrame(IBLOB *indiBlob)
{
    static bool skipped = false;
    uint8_t *blob = static_cast<uint8_t *>(indiBlob->blob);
    size_t size = indiBlob->size;

    if (!strcmp(indiBlob->format, ".fits"))
    {
        indiFillFrameFITS(blob, size);
        skipped = false;
    }
    else if (!strcmp(indiBlob->format, ".stream_jpg") || !strcmp(indiBlob->format, ".jpg"))
    {
        indiFillFrameJpeg(blob, size);
        skipped = false;
    }
    else if (!strcmp(indiBlob->format, ".stream")) {
        switch(devices[current_ccd].video_method)
        {
            case ObsIndiClient::Device::METHOD_CAPTURE: {
                switch (devices[current_ccd].capture_format) {
                    case ObsIndiClient::Device::FORMAT_FITS: {
                        indiFillFrameFITS(blob, size);
                        skipped = false;
                        break;
                    }
                    default: {
                        if (!skipped) {
                            blog(LOG_INFO, "Unsupported format, skipping");
                        }
                        skipped = true;
                        break;
                    }
                }
                break;
            }
            case ObsIndiClient::Device::METHOD_STREAM : {
                switch (devices[current_ccd].stream_format) {
                    case ObsIndiClient::Device::FORMAT_MJPEG: {
                        indiFillFrameJpeg(blob, size);
                        skipped = false;
                        break;
                    }
                    case ObsIndiClient::Device::FORMAT_RAW: {
                        if (!skipped)
                            blog(LOG_INFO, "Unsupported format RAW, skipping");
                        skipped = true;
                        break;
                    }
                }
                break;
            }
        }
    } else {
        if (!skipped) {
            blog(LOG_INFO, "Unknown blob format %s, skipping", indiBlob->format);
        }
        skipped = true;
        return;
    }
}

void ObsIndiClient::dummyFillFrame()
{
    // Display dummy image
    width = no_signal_image.width;
    height = no_signal_image.height;
    blog(LOG_INFO, "Fill dummy frame %ix%i", width, height);
    rgba_frame.resize(width * height * 4);
    memcpy(rgba_frame.data(), no_signal_image.pixel_data, width * height * 4);
}

void ObsIndiClient::newDevice(INDI::BaseDevice baseDevice)
{
    std::string device_name(baseDevice.getDeviceName());
    std::cout << "New device: " << device_name << std::endl;
    Device dev;
    dev.device_name = device_name;
    dev.isCCD = false;
    devices[device_name] = dev;
}

void ObsIndiClient::removeDevice(INDI::BaseDevice baseDevice)
{
    std::string device_name(baseDevice.getDeviceName());
    auto it = devices.find(device_name);
    if (it != devices.end())
        devices.erase(it);
    std::cout << "Remove device: " << device_name << std::endl;
    if (current_ccd == device_name)
        current_ccd = "";
}

void ObsIndiClient::handleProperty(INDI::Property property)
{
    std::string device_name = property.getDeviceName();
    std::string property_name = property.getName();
    auto ptype = property.getType();
    switch (ptype) {
        case INDI_BLOB: {
            std::cout << "blob property: " << property_name
                      << " for device " << device_name
                      << " current_ccd = " << current_ccd
                      << std::endl;
            if (current_ccd == device_name) {
                devices[device_name].property_name = property_name;
                devices[device_name].isCCD = true;
                std::cout << "Enable blob data on " << property_name << ":" << device_name << std::endl;
                setBLOBMode(B_ALSO, device_name.c_str(), property_name.c_str());
            } else {
                INDI::BaseDevice dev = getDevice(device_name.c_str());
                if (!(dev.getDriverInterface() & INDI::BaseDevice::CCD_INTERFACE))
                    break;
                devices[device_name].property_name = property_name;
                devices[device_name].isCCD = true;
            }
            break;
        }
        case INDI_NUMBER: {
            //std::cout << "number property: " << property_name << " for device " << device_name << std::endl;
            INumberVectorProperty *numberProp = property.getNumber();
            for (int i = 0; i < numberProp->nnp; i++)
            {
                std::string name = numberProp->np[i].name;
                int value = numberProp->np[i].value;
                if (property_name == "CCD_FRAME") {
                    if (name == "WIDTH")
                        devices[device_name].width = value;
                    else if (name == "HEIGHT")
                        devices[device_name].height = value;
                }
            }
            break;
        }
        case INDI_TEXT : {
            std::cout << "text property: " << property_name << " for device " << device_name << std::endl;
            ITextVectorProperty *textProp = property.getText();
            for (int i = 0; i < textProp->ntp; i++)
            {
                std::cout << "    " << textProp->tp[i].name << " = " << textProp->tp[i].text << std::endl;
            }
            break;
        }
        case INDI_SWITCH : {
            std::cout << "switch property: " << property_name << " for device " << device_name << std::endl;
            ISwitchVectorProperty *switchProp = property.getSwitch();
            for (int i = 0; i < switchProp->nsp; i++)
            {
                std::string name = switchProp->sp[i].name;
                int value = switchProp->sp[i].s;
                std::cout << "    " << name << " = " << value << std::endl;
                if (property_name == "CCD_TRANSFER_FORMAT") {
                    if (name == "FORMAT_FITS" && value)
                        devices[device_name].capture_format = ObsIndiClient::Device::FORMAT_FITS;
                    else if (name == "FORMAT_NATIVE" && value)
                        devices[device_name].capture_format = ObsIndiClient::Device::FORMAT_NATIVE;
                    else if (name == "FORMAT_XISF" && value)
                        devices[device_name].capture_format = ObsIndiClient::Device::FORMAT_XISF;
                } else if (property_name == "CCD_VIDEO_STREAM") {
                    // TODO: some drivers can send stream 1 frame after streaming off. So if we receive frame exactly
                    // after stream off, drop it
                    if (name == "STREAM_ON" && value)
                        devices[device_name].video_method = ObsIndiClient::Device::METHOD_STREAM;
                    else if (name == "STREAM_OFF" && value)
                        devices[device_name].video_method = ObsIndiClient::Device::METHOD_CAPTURE;
                } else if (property_name == "CCD_STREAM_ENCODER") {
                    if (name == "RAW" && value) {
                        devices[device_name].stream_format = ObsIndiClient::Device::FORMAT_RAW;
                    } else if (name == "MJPEG" && value) {
                        devices[device_name].stream_format = ObsIndiClient::Device::FORMAT_MJPEG;
                    }
                }
            }
            break;
        }
        case INDI_LIGHT : {
            std::cout << "light property: " << property_name << " for device " << device_name << std::endl;
            break;
        }
        default:
            break;
    }
}

void ObsIndiClient::newProperty(INDI::Property property)
{
    handleProperty(property);
}

void ObsIndiClient::updateProperty(INDI::Property property)
{
    // Called when a property is updated
    std::string property_name = property.getName();
    std::string device_name = property.getDeviceName();

    auto prop = std::make_pair(property_name, device_name);

    if (device_name == current_ccd && devices[current_ccd].property_name == property_name && devices[current_ccd].isCCD)
    {
        auto blobProp = property.getBLOB();
        for (auto blob : *blobProp)
            processBLOB(&blob);
    } else {
        handleProperty(property);
    }
}

void ObsIndiClient::removeProperty(INDI::Property property)
{
    std::string device_name = property.getDeviceName();
    std::string property_name = property.getName();

    std::cout << "Property removed: " << property_name << " for device " << device_name << std::endl;
}

void ObsIndiClient::newMessage(INDI::BaseDevice dp, int messageID)
{
    //std::cout << "New message from " << dp.getDeviceName() << ": " << dp.messageQueue(messageID) << std::endl;
}

void ObsIndiClient::processBLOB(IBLOB *indiBlob)
{
    std::lock_guard<std::mutex> lock(frame_mutex);

    if (indiBlob == nullptr)
    {
        dummyFillFrame();
    }
    else
    {
        indiFillFrame(indiBlob);
    }
}
