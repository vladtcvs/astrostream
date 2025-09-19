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
#include <list>
#include <mutex>
#include <stdint.h>

class ObsIndiClient : public INDI::BaseClient
{
private:
    uint8_t fitsGetPixel(const std::vector<uint8_t>& data, size_t w, size_t h, int bitpix, int x, int y, int plane) const;

    void indiFillFrameJpeg(const uint8_t *data, size_t size);
    void indiFillFrameFITS(uint8_t *data, size_t size);
public:
    ObsIndiClient(std::mutex &mutex) : frame_mutex(mutex) {}

    std::pair<std::string, std::string> ccddev_decode(const std::string& ccddev);
    std::string ccddev_encode(const std::pair<std::string, std::string> &ccd);

    int indiFrameWidth() const;
    int indiFrameHeight() const;
    const std::vector<uint8_t>& indiFrame() const;

    std::list<std::pair<std::string, std::string>> indiCCDs() const;
    std::pair<std::string, std::string> indiCurrentCCD() const;
    void selectCCD(const std::string &ccddev);
    void selectCCD(const std::pair<std::string, std::string> &ccd);

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
    std::list<std::string> devices;
    size_t width, height;
    std::vector<uint8_t> rgba_frame;
    std::pair<std::string, std::string> current_ccd;
    std::list<std::pair<std::string, std::string>> ccds;
};
