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

std::list<std::pair<std::string, std::string>> ObsIndiClient::indiCCDs() const
{
    return ccds;
}

std::pair<std::string, std::string> ObsIndiClient::indiCurrentCCD() const
{
    return current_ccd;
}

bool ObsIndiClient::connectServer()
{
    bool res = INDI::BaseClient::connectServer();
    dummyFillFrame();
    return res;
}

bool ObsIndiClient::disconnectServer(int exit_code)
{
    ccds.clear();
    current_ccd = std::make_pair("", "");
    dummyFillFrame();
    return INDI::BaseClient::disconnectServer(exit_code);
}

void ObsIndiClient::selectCCD(const std::string &ccddev)
{
    auto ccd = ccddev_decode(ccddev);
    current_ccd = ccd;
}

void ObsIndiClient::selectCCD(const std::pair<std::string, std::string> &ccd)
{
    if (current_ccd == ccd)
        return;

    auto property_name = current_ccd.first;
    auto device_name = current_ccd.second;
    if (property_name != "" && device_name != "") {
        std::cout << "Disable blob data on " << property_name << ":" << device_name << std::endl;
        setBLOBMode(B_NEVER, device_name.c_str(), property_name.c_str());
    }

    current_ccd = ccd;

    property_name = current_ccd.first;
    device_name = current_ccd.second;
    if (property_name != "" && device_name != "") {
        std::cout << "Enable blob data on " << property_name << ":" << device_name << std::endl;
        setBLOBMode(B_ALSO, device_name.c_str(), property_name.c_str());
    }
    dummyFillFrame();
}

std::pair<std::string, std::string> ObsIndiClient::ccddev_decode(const std::string& ccddev)
{
    size_t pos = ccddev.find(":");
    std::pair<std::string, std::string> newccd;
    if (pos == ccddev.npos)
        newccd    = std::make_pair("", "");
    else
        newccd    = std::make_pair(ccddev.substr(0, pos), ccddev.substr(pos + 1));
    return newccd;
}

std::string ObsIndiClient::ccddev_encode(const std::pair<std::string, std::string> &ccd)
{
    return ccd.first + ":" + ccd.second;
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
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);

    // Set up JPEG data source
    jpeg_mem_src(&cinfo, data, size);

    // Read JPEG header
    jpeg_read_header(&cinfo, true);

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
    else if (!strcmp(indiBlob->format, ".stream_jpg"))
    {
        indiFillFrameJpeg(blob, size);
        skipped = false;
    }
    else
    {
        if (!skipped)
            blog(LOG_INFO, "Unknown blob format %s, skipping", indiBlob->format);
        skipped = true;
        return;
    }
}

void ObsIndiClient::dummyFillFrame()
{
    // Display dummy image
    // TODO: add "no signal" text
    width = no_signal_image.width;
    height = no_signal_image.height;
    blog(LOG_INFO, "Fill dummy frame %ix%i", width, height);
    rgba_frame.resize(width * height * 4);
    memcpy(rgba_frame.data(), no_signal_image.pixel_data, width * height * 4);
}

void ObsIndiClient::newDevice(INDI::BaseDevice baseDevice)
{
    std::string name(baseDevice.getDeviceName());
    devices.push_back(name);
    std::cout << "New device: " << name << std::endl;
}

void ObsIndiClient::removeDevice(INDI::BaseDevice baseDevice)
{
    std::string name(baseDevice.getDeviceName());
    auto it = std::find(devices.begin(), devices.end(), name);
    if (it != devices.end())
        devices.erase(it);
    std::cout << "Remove device: " << name << std::endl;
}

void ObsIndiClient::newProperty(INDI::Property property)
{
    std::string device_name = property.getDeviceName();
    std::string property_name = property.getName();
    auto ptype = property.getType();
    std::cout << "New property: " << property_name << " for device " << device_name << " (" << ptype << ")" << std::endl;
    if (ptype != INDI_BLOB)
        return;

    INDI::BaseDevice dev = getDevice(device_name.c_str());
    if (!(dev.getDriverInterface() & INDI::BaseDevice::CCD_INTERFACE))
        return;

    auto ccd = std::make_pair(property_name, device_name);
    ccds.push_back(ccd);
    if (ccd == current_ccd)
    {
        std::cout << "Enable blob data on " << property_name << ":" << device_name << std::endl;
        setBLOBMode(B_ALSO, device_name.c_str(), property_name.c_str());
    }
}

void ObsIndiClient::updateProperty(INDI::Property property)
{
    // Called when a property is updated
    std::string property_name = property.getName();
    std::string device_name = property.getDeviceName();

    auto prop = std::make_pair(property_name, device_name);

    if (prop == current_ccd)
    {
        // std::cout << "CCD updated: " << property.getName() << " for device " << property.getDeviceName() << std::endl;
        auto blobProp = property.getBLOB();
        for (auto blob : *blobProp)
            processBLOB(&blob);
    }
}

void ObsIndiClient::removeProperty(INDI::Property property)
{
    std::string device_name = property.getDeviceName();

    // Called when a property is removed
    std::string property_name = property.getName();

    std::cout << "Property removed: " << property_name << " for device " << device_name << std::endl;
    auto ccd = std::make_pair(property_name, device_name);
    auto it = std::find(ccds.begin(), ccds.end(), ccd);
    if (it != ccds.end())
    {
        ccds.erase(it);
    }
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
