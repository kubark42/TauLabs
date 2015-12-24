/**
 ******************************************************************************
 *
 * @file       snapdragonflight.cpp
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2015
 *
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup Boards_CyPhy CyPhy Works boards support Plugin
 * @{
 * @brief Plugin to support boards by CyPhy Works
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "snapdragonflight.h"

#include <uavobjectmanager.h>
#include "uavobjectutil/uavobjectutilmanager.h"
#include <extensionsystem/pluginmanager.h>

#include "hwsnapdragonflight.h"

/**
 * @brief SnapdragonFlight::SnapdragonFlight
 *  This is the SnapdragonFlight board definition
 */
SnapdragonFlight::SnapdragonFlight(void)
{
    // Initialize our USB Structure definition here:
    USBInfo board;
    // XXX fix these
    board.vendorID = 0x20A0;
    board.productID = 0x415A;

    setUSBInfo(board);

    boardType = 0xFA;

    // Define the bank of channels that are connected to a given timer
    channelBanks.resize(5);
    channelBanks[0] = QVector<int> () << 1 << 2;
    channelBanks[1] = QVector<int> () << 3 << 4;
    channelBanks[2] = QVector<int> () << 5 << 6;
    channelBanks[3] = QVector<int> () << 7 << 8;
    channelBanks[4] = QVector<int> () << 9 << 10;
}

SnapdragonFlight::~SnapdragonFlight()
{

}

QString SnapdragonFlight::shortName()
{
    return QString("SnapdragonFlight");
}

QString SnapdragonFlight::boardDescription()
{
    return QString("SnapdragonFlight flight control rev. 1 by Qualcomm");
}

//! Return which capabilities this board has
bool SnapdragonFlight::queryCapabilities(BoardCapabilities capability)
{
    switch(capability) {
    case BOARD_CAPABILITIES_GYROS:
        return true;
    case BOARD_CAPABILITIES_ACCELS:
        return true;
    case BOARD_CAPABILITIES_MAGS:
        return true;
    case BOARD_CAPABILITIES_BAROS:
        return true;
    case BOARD_CAPABILITIES_RADIO:
        return false;
    }
    return false;
}


/**
 * @brief SnapdragonFlight::getSupportedProtocols
 *  TODO: this is just a stub, we'll need to extend this a lot with multi protocol support
 * @return
 */
QStringList SnapdragonFlight::getSupportedProtocols()
{

    return QStringList("uavtalk");
}

QPixmap SnapdragonFlight::getBoardPicture()
{
    return QPixmap(":/cyphy/images/snapdragonflight.png");
}

QString SnapdragonFlight::getHwUAVO()
{
    return "HwSnapdragonFlight";
}

int SnapdragonFlight::queryMaxGyroRate()
{
    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    UAVObjectManager *uavoManager = pm->getObject<UAVObjectManager>();
    HwSnapdragonFlight *hwSnapdragonFlight = HwSnapdragonFlight::GetInstance(uavoManager);
    Q_ASSERT(hwSnapdragonFlight);
    if (!hwSnapdragonFlight)
        return 0;

    HwSnapdragonFlight::DataFields settings = hwSnapdragonFlight->getData();

    switch(settings.GyroRange) {
    case HwSnapdragonFlight::GYRORANGE_250:
        return 250;
    case HwSnapdragonFlight::GYRORANGE_500:
        return 500;
    case HwSnapdragonFlight::GYRORANGE_1000:
        return 1000;
    case HwSnapdragonFlight::GYRORANGE_2000:
        return 2000;
    default:
        return 500;
    }
}
