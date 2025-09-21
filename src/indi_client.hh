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

#pragma once

#include <baseclient.h>
#include <basedevice.h>
#include <indidevapi.h>
#include <vector>
#include <map>
#include <list>
#include <optional>
#include <mutex>
#include <stdint.h>

class ObsIndiClient : public INDI::BaseClient
{
public:
    class Device {
    public:
        bool isCCD = false;
        bool blobEnabled = false;

        enum video_method_e {
            METHOD_CAPTURE = 0,
            METHOD_STREAM = 1,
        } video_method = METHOD_CAPTURE;

        enum capture_format_e {
            FORMAT_FITS = 0,
            FORMAT_NATIVE = 1,
            FORMAT_XISF = 2,
        } capture_format = FORMAT_NATIVE;

        enum stream_format_e {
            FORMAT_MJPEG = 0,
            FORMAT_RAW = 1,
        } stream_format = FORMAT_RAW;
        std::string raw_format;

        size_t capture_width = 0, capture_height = 0;
        size_t stream_width = 0, stream_height = 0;
        std::string device_name;
        std::string property_name;
    public:
        bool operator == (const Device& other)
        {
            return device_name == other.device_name;
        }
    };
private:
    uint8_t fitsGetPixel(const std::vector<uint8_t>& data, size_t w, size_t h, int bitpix, int x, int y, int plane) const;

    void indiFillFrameJpeg(const uint8_t *data, size_t size);
    void indiFillFrameFITS(uint8_t *data, size_t size);
    void handleProperty(INDI::Property property);
public:
    ObsIndiClient(std::mutex &mutex) : frame_mutex(mutex) {}

    bool connectServer() override;
    bool disconnectServer(int exit_code = 0) override;

    int indiFrameWidth() const;
    int indiFrameHeight() const;
    const std::vector<uint8_t>& indiFrame() const;

    std::list<std::string> indiCCDs() const;
    std::string indiCurrentCCD() const;
    void selectCCD(const std::string &ccd);

    void indiFillFrame(IBLOB *indiBlob);
    void dummyFillFrame();
    void processBLOB(IBLOB *indiBlob);

    void newDevice(INDI::BaseDevice baseDevice) override;
    void removeDevice(INDI::BaseDevice baseDevice) override;
    void newProperty(INDI::Property property) override;
    void updateProperty(INDI::Property property) override;
    void removeProperty(INDI::Property property) override;
    void newMessage(INDI::BaseDevice dp, int messageID) override;

private:
    std::mutex &frame_mutex;
    size_t width = 0;
    size_t height = 0;
    std::vector<uint8_t> data;
    std::vector<uint8_t> raw_frame;
    std::vector<uint8_t> rgba_frame;
    std::string current_ccd;
    std::map<std::string, Device> devices;
};
