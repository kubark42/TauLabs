/**
 ******************************************************************************
 *
 * @file       uavsettingsimportexportfactory.h
 * @author     (C) 2011 The OpenPilot Team, http://www.openpilot.org
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup UAVSettingsImportExport UAVSettings Import/Export Plugin
 * @{
 * @brief UAVSettings Import/Export Plugin
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
#ifndef UAVSETTINGSIMPORTEXPORTFACTORY_H
#define UAVSETTINGSIMPORTEXPORTFACTORY_H
#include "uavsettingsimportexport_global.h"
#include "uavobjectutil/uavobjectutilmanager.h"
#include "uavtalk/telemetrymanager.h"
#include "../../../../../build/ground/gcs/gcsversioninfo.h"

class UAVSETTINGSIMPORTEXPORT_EXPORT UAVSettingsImportExportFactory : public QObject
{
    Q_OBJECT

public:
    UAVSettingsImportExportFactory(QObject *parent = 0);
    ~UAVSettingsImportExportFactory();
    void uploadUAVSettings();

private:
    enum storedData { Settings, Data, Both };
    enum whichCache { CACHE_NEWEST, CACHE_OLDEST };
    class QNetworkAccessManager *networkAccessManager;
    QString serverURL;
    QString createXMLDocument(const enum storedData, const bool fullExport);
    QString md5Checksum(QString str);
    QString getUAVSettingsCachePath();
    QString findCacheFilename(QString pathName, const enum whichCache which);
    QString retrieveCacheFile(QString cacheName, QString CPUSerial);
    QString retrieveCacheFile(QString CPUSerial);
    bool isDirectoryEmpty(QString directoryName);
    bool POSTCacheFile(QString pathName, QString CPUSerial);
    bool verifyCacheUpload(QString pathName, QString CPUSerial);
    bool createServerDirectory(QString directoryName);

public slots:
    void backupUAVSettings();

private slots:
    void importUAVSettings();
    void exportUAVSettings();
    void exportUAVData();

signals:
    void importAboutToBegin();
    void importEnded();
};

#endif // UAVSETTINGSIMPORTEXPORTFACTORY_H
