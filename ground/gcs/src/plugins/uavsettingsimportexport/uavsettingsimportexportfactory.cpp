/**
 ******************************************************************************
 *
 * @file       uavsettingsimportexportfactory.cpp
 * @author     (C) 2011 The OpenPilot Team, http://www.openpilot.org
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2014
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

#include "uavsettingsimportexportfactory.h"
#include <QtPlugin>
#include <QStringList>
#include <QDebug>
#include <QCheckBox>
#include "importsummary.h"

// for menu item
#include <coreplugin/coreconstants.h>
#include <coreplugin/actionmanager/actionmanager.h>
#include <coreplugin/icore.h>
#include <QKeySequence>

// for UAVObjects
#include "uavdataobject.h"
#include "uavobjectmanager.h"
#include "extensionsystem/pluginmanager.h"

// for XML object
#include <QDomDocument>
#include <QXmlQuery>

// for file dialog and error messages
#include <QFileDialog>
#include <QMessageBox>

// for UAVObject settings backup
#include "utils/pathutils.h"

UAVSettingsImportExportFactory::~UAVSettingsImportExportFactory()
{
    // Do nothing
}

UAVSettingsImportExportFactory::UAVSettingsImportExportFactory(QObject *parent):QObject(parent)
{

    // Add Menu entry
    Core::ActionManager *am = Core::ICore::instance()->actionManager();
    Core::ActionContainer *ac = am->actionContainer(Core::Constants::M_FILE);
    Core::Command *cmd = am->registerAction(new QAction(this),
                                            "UAVSettingsImportExportPlugin.UAVSettingsExport",
                                            QList<int>() <<
                                            Core::Constants::C_GLOBAL_ID);
    cmd->setDefaultKeySequence(QKeySequence("Ctrl+E"));
    cmd->action()->setText(tr("Export UAV Settings..."));
    ac->addAction(cmd, Core::Constants::G_FILE_SAVE);
    connect(cmd->action(), SIGNAL(triggered(bool)), this, SLOT(exportUAVSettings()));

    cmd = am->registerAction(new QAction(this),
                             "UAVSettingsImportExportPlugin.UAVSettingsImport",
                             QList<int>() <<
                             Core::Constants::C_GLOBAL_ID);
    cmd->setDefaultKeySequence(QKeySequence("Ctrl+I"));
    cmd->action()->setText(tr("Import UAV Settings..."));
    ac->addAction(cmd, Core::Constants::G_FILE_SAVE);
    connect(cmd->action(), SIGNAL(triggered(bool)), this, SLOT(importUAVSettings()));

    ac = am->actionContainer(Core::Constants::M_HELP);
    cmd = am->registerAction(new QAction(this),
                             "UAVSettingsImportExportPlugin.UAVDataExport",
                             QList<int>() <<
                             Core::Constants::C_GLOBAL_ID);
    cmd->action()->setText(tr("Export UAV Data..."));
    ac->addAction(cmd, Core::Constants::G_HELP_HELP);
    connect(cmd->action(), SIGNAL(triggered(bool)), this, SLOT(exportUAVData()));

}

// Slot called by the menu manager on user action
void UAVSettingsImportExportFactory::importUAVSettings()
{
    // ask for file name
    QString fileName;
    QString filters = tr("UAVObjects XML files (*.uav);; XML files (*.xml)");
    fileName = QFileDialog::getOpenFileName(0, tr("Import UAV Settings"), "", filters);
    if (fileName.isEmpty()) {
        return;
    }

    // Now open the file
    QFile file(fileName);
    QDomDocument doc("UAVObjects");
    file.open(QFile::ReadOnly|QFile::Text);
    if (!doc.setContent(file.readAll())) {
        QMessageBox msgBox;
        msgBox.setText(tr("File Parsing Failed."));
        msgBox.setInformativeText(tr("This file is not a correct XML file"));
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
        return;
    }
    file.close();

    // find the root of settings subtree
    emit importAboutToBegin();
    qDebug()<<"Import about to begin";

    QDomElement root = doc.documentElement();
    if (root.tagName() == "uavobjects") {
        root = root.firstChildElement("settings");
    }
    if (root.isNull() || (root.tagName() != "settings")) {
        QMessageBox msgBox;
        msgBox.setText(tr("Wrong file contents"));
        msgBox.setInformativeText(tr("This file does not contain correct UAVSettings"));
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
        return;
    }

    // We are now ok: setup the import summary dialog & update it as we
    // go along.
    ImportSummaryDialog swui((QWidget*)Core::ICore::instance()->mainWindow());

    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    UAVObjectManager *objManager = pm->getObject<UAVObjectManager>();
    swui.show();

    QDomNode node = root.firstChild();
    while (!node.isNull()) {
        QDomElement e = node.toElement();
        if (e.tagName() == "object") {

            //  - Read each object
            QString uavObjectName  = e.attribute("name");
            uint uavObjectID = e.attribute("id").toUInt(NULL,16);

            // Sanity Check:
            UAVObject *obj = objManager->getObject(uavObjectName);
            UAVDataObject *dobj = dynamic_cast<UAVDataObject*>(obj);
            if (obj == NULL) {
                // This object is unknown!
                qDebug() << "Object unknown:" << uavObjectName << uavObjectID;
                swui.addLine(uavObjectName, "Error (Object unknown)", false);
            } else if(dobj && !dobj->getIsPresentOnHardware()) {
                swui.addLine(uavObjectName, "Error (Object not present on hw)", false);
            } else {
                //  - Update each field
                //  - Issue and "updated" command
                bool error = false;
                bool setError = false;
                QDomNode field = node.firstChild();
                while(!field.isNull()) {
                    QDomElement f = field.toElement();
                    if (f.tagName() == "field") {
                        UAVObjectField *uavfield = obj->getField(f.attribute("name"));
                        if (uavfield) {
                            QStringList list = f.attribute("values").split(",");
                            if (list.length() == 1) {
                                if (false == uavfield->checkValue(f.attribute("values"))) {
                                    qDebug() << "checkValue returned false on: " << uavObjectName << f.attribute("values");
                                    setError = true;
                                } else {
                                    uavfield->setValue(f.attribute("values"));
                                }
                            } else {
                                // This is an enum:
                                int i = 0;
                                QStringList list = f.attribute("values").split(",");
                                foreach (QString element, list) {
                                    if (false == uavfield->checkValue(element, i)) {
                                        qDebug() << "checkValue(list) returned false on: " << uavObjectName << list;
                                        setError = true;
                                    } else {
                                        uavfield->setValue(element,i);
                                    }
                                    i++;
                                }
                            }
                        } else {
                            error = true;
                        }
                    }
                    field = field.nextSibling();
                }
                obj->updated();

                if (error) {
                    swui.addLine(uavObjectName, "Warning (Object field unknown)", true);
                } else if (uavObjectID != obj->getObjID()) {
                    qDebug() << "Mismatch for Object " << uavObjectName << uavObjectID << " - " << obj->getObjID();
                    swui.addLine(uavObjectName, "Warning (ObjectID mismatch)", true);
                } else if (setError) {
                    swui.addLine(uavObjectName, "Warning (Objects field value(s) invalid)", false);
                } else {
                    swui.addLine(uavObjectName, "OK", true);
                }
            }
        }
        node = node.nextSibling();
    }
    qDebug() << "End import";
    swui.exec();
}

/**
 * @brief UAVSettingsImportExportFactory::md5Checksum calculates md5 checksum of a QString
 * @param str
 * @return md5checksum of str
 */
QString UAVSettingsImportExportFactory::md5Checksum(QString str)
{
    QTextCodec *codec = QTextCodec::codecForName("ISO 8859-1"); // encoding scheme was chosen arbitrarily
    QByteArray utf8 = codec->fromUnicode(str);
    QString checksum = QString(QCryptographicHash::hash(utf8, QCryptographicHash::Md5).toHex());
    return checksum;
}

// Create an XML document from UAVObject database
QString UAVSettingsImportExportFactory::createXMLDocument(const enum storedData what, const bool fullExport)
{
    // generate an XML first (used for all export formats as a formatted data source)
    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    UAVObjectManager *objManager = pm->getObject<UAVObjectManager>();

    // create an XML root
    QDomDocument doc("UAVObjects");
    QDomElement root = doc.createElement("uavobjects");
    doc.appendChild(root);

    // add hardware, firmware and GCS version info
    QDomElement versionInfo = doc.createElement("version");
    root.appendChild(versionInfo);

    UAVObjectUtilManager *utilMngr = pm->getObject<UAVObjectUtilManager>();
    deviceDescriptorStruct board;
    utilMngr->getBoardDescriptionStruct(board);

    QDomElement hw = doc.createElement("hardware");
    hw.setAttribute("type", QString().setNum(board.boardType, 16));
    hw.setAttribute("revision", QString().setNum(board.boardRevision, 16));
    hw.setAttribute("serial", QString(utilMngr->getBoardCPUSerial().toHex()));
    versionInfo.appendChild(hw);

    QDomElement fw = doc.createElement("firmware");
    fw.setAttribute("date", board.gitDate);
    fw.setAttribute("hash", board.gitHash);
    fw.setAttribute("tag", board.gitTag);
    versionInfo.appendChild(fw);

    QString gcsRevision = QString::fromLatin1(Core::Constants::GCS_REVISION_STR);
    QString gcsGitDate = gcsRevision.mid(gcsRevision.indexOf(" ") + 1, 14);
    QString gcsGitHash = gcsRevision.mid(gcsRevision.indexOf(":") + 1, 8);
    QString gcsGitTag = gcsRevision.left(gcsRevision.indexOf(":"));

    QDomElement gcs = doc.createElement("gcs");
    gcs.setAttribute("date", gcsGitDate);
    gcs.setAttribute("hash", gcsGitHash);
    gcs.setAttribute("tag", gcsGitTag);
    versionInfo.appendChild(gcs);

    // create settings and/or data elements
    QDomElement settings = doc.createElement("settings");
    QDomElement data = doc.createElement("data");

    switch (what)
    {
    case Settings:
        root.appendChild(settings);
        break;
    case Data:
        root.appendChild(data);
        break;
    case Both:
        root.appendChild(data);
        root.appendChild(settings);
        break;
    }

    // iterate over settings objects
    QVector< QVector<UAVDataObject*> > objList = objManager->getDataObjectsVector();
    foreach (QVector<UAVDataObject*> list, objList) {
        foreach (UAVDataObject *obj, list) {
            if(!obj->getIsPresentOnHardware())
                continue;
            if (((what == Settings) && obj->isSettings()) ||
                    ((what == Data) && !obj->isSettings()) ||
                    (what == Both)) {

                // add each object to the XML
                QDomElement o = doc.createElement("object");
                o.setAttribute("name", obj->getName());
                o.setAttribute("id", QString("0x")+ QString().setNum(obj->getObjID(),16).toUpper());
                if (fullExport) {
                    QDomElement d = doc.createElement("description");
                    QDomText t = doc.createTextNode(obj->getDescription().remove("@Ref ", Qt::CaseInsensitive));
                    d.appendChild(t);
                    o.appendChild(d);
                }

                // iterate over fields
                QList<UAVObjectField*> fieldList = obj->getFields();

                foreach (UAVObjectField* field, fieldList) {
                    QDomElement f = doc.createElement("field");

                    // iterate over values
                    QString vals;
                    quint32 nelem = field->getNumElements();
                    for (unsigned int n = 0; n < nelem; ++n) {
                        vals.append(QString("%1,").arg(field->getValue(n).toString()));
                    }
                    vals.chop(1);

                    f.setAttribute("name", field->getName());
                    f.setAttribute("values", vals);
                    if (fullExport) {
                        f.setAttribute("type", field->getTypeAsString());
                        f.setAttribute("units", field->getUnits());
                        f.setAttribute("elements", nelem);
                        if (field->getType() == UAVObjectField::ENUM) {
                            f.setAttribute("options", field->getOptions().join(","));
                        }
                    }
                    o.appendChild(f);
                }

                // append to the settings or data element
                if (obj->isSettings())
                    settings.appendChild(o);
                else
                    data.appendChild(o);
            }
        }
    }

    // This sorts the XML <object> children's <name=".."> attribute by alphabetical order. This
    // is particularly helpful when comparing *.uav files to each other.
    QString preliminaryXMLDoc = doc.toString(4);
    QString alphabetizedXMLDoc;

    QString xmlAlpheticalSorter(" \
        <xsl:stylesheet version=\"2.0\" xmlns:xsl=\"http://www.w3.org/1999/XSL/Transform\"> \
          <xsl:output method=\"xml\" indent=\"yes\" omit-xml-declaration=\"no\"/> \
          <xsl:strip-space elements=\"*\"/> \
          \
          <xsl:template match=\"@* | node()\"> \
            <xsl:copy> \
              <xsl:apply-templates select=\"@* | node()\"/> \
            </xsl:copy> \
          </xsl:template> \
          \
          <xsl:template match=\"settings\"> \
            <xsl:copy> \
              <xsl:apply-templates select=\"@*\" /> \
              <xsl:apply-templates select=\"object\"> \
                <xsl:sort select=\"@name\" data-type=\"text\"/> \
              </xsl:apply-templates> \
            </xsl:copy> \
          </xsl:template> \
        </xsl:stylesheet> \
    ");

    QXmlQuery query(QXmlQuery::XSLT20);
    query.setFocus(preliminaryXMLDoc);
    query.setQuery(xmlAlpheticalSorter);
    query.evaluateTo(&alphabetizedXMLDoc);

    return alphabetizedXMLDoc;
}

// Slot called by the menu manager on user action
void UAVSettingsImportExportFactory::exportUAVSettings()
{
    // ask for file name
    QString fileName;
    QString filters = tr("UAVObjects XML files (*.uav)");

    fileName = QFileDialog::getSaveFileName(0, tr("Save UAVSettings File As"), "", filters);
    if (fileName.isEmpty()) {
        return;
    }

    // If the filename ends with .xml, we will do a full export, otherwise, a simple export
    bool fullExport = false;
    if (fileName.endsWith(".xml")) {
        fullExport = true;
    } else if (!fileName.endsWith(".uav")) {
        fileName.append(".uav");
    }

    // generate an XML first (used for all export formats as a formatted data source)
    QString xml = createXMLDocument(Settings, fullExport);

    // save file
    QFile file(fileName);
    if (file.open(QIODevice::WriteOnly) &&
            (file.write(xml.toLatin1()) != -1)) {
        file.close();
    } else {
        QMessageBox::critical(0,
                              tr("UAV Settings Export"),
                              tr("Unable to save settings: ") + fileName,
                              QMessageBox::Ok);
        return;
    }

    QMessageBox msgBox;
    msgBox.setText(tr("Settings saved."));
    msgBox.setStandardButtons(QMessageBox::Ok);
    msgBox.exec();
}

/**
 * @brief UAVSettingsImportExportFactory::backupUAVSettings Creates cache file containing a vehicle's
 * current UAVSettings. This function is a slot called when a new session is started.
 */
void UAVSettingsImportExportFactory::backupUAVSettings()
{
    bool fullExport = true;

    // Determine filename and directory of the UAVSettings cache
    QString pathName = getUAVSettingsCachePath();
    quint32 currentTime = QDateTime::currentDateTime().toTime_t();
    QString newFileName = QDir(pathName).filePath(QString::number(currentTime) + ".uav");

    // If the UAVSettings cache directory doesn't exist, create it. Otherwise determine the most recent cache file.
    QString mrFileName;  // most recent cache file
    QDir dir(pathName);
    if (!dir.exists()) {
        if (!dir.mkpath(".")) {
            qDebug()<<"Unable to create directory '" << pathName << "' for UAVSettings cache.";
            return;
        }
    } else {
        mrFileName = findCache(pathName, Newest);
    }

    // Generate an XML string containing all the Settings UAVObjects
    QString currentSettings = createXMLDocument(Settings, fullExport);

    // Compare the MD5 checksum of the current settings to the MD5 checksum of the most recent
    // UAVSettings cache file to see if settings have changed
    bool createNewFile = true;
    if (!mrFileName.isNull()) {
        QFile mrFile(mrFileName);

        if (mrFile.open(QFile::ReadOnly)) {
            // Read contents of the most recent UAVSettings cache file
            QTextStream in(&mrFile);
            QString mrSettings = in.readAll();
            mrFile.close();

            // compare MD5 checksums of current settings and the most recent settings
            QString mrSettingsMd5 = md5Checksum(mrSettings);
            QString currentSettingsMd5 = md5Checksum(currentSettings);

            if (currentSettingsMd5 == mrSettingsMd5) {
               // settings have not changed, no need to save a new cache
               createNewFile = false;
            }
        }
    }

    // If UAVSettings have changed, then save new UAVSettings cache file
    if (createNewFile) {
        QFile newFile(newFileName);
        if (newFile.open(QIODevice::WriteOnly) &&
                (newFile.write(currentSettings.toLatin1()) != -1)) {
            newFile.close();
        } else {
            qDebug() << "Unable to backup UAV settings to" << newFileName;
        }
    }
    return;
}

// Slot called by the menu manager on user action
void UAVSettingsImportExportFactory::exportUAVData()
{
    if (QMessageBox::question(0, tr("Are you sure?"),
                              tr("This option is only useful for passing your current "
                                 "system data to the technical support staff. "
                                 "Do you really want to export?"),
                              QMessageBox::Ok | QMessageBox::Cancel,
                              QMessageBox::Ok) != QMessageBox::Ok) {
        return;
    }

    // ask for file name
    QString fileName;
    QString filters = tr("UAVObjects XML files (*.uav)");

    fileName = QFileDialog::getSaveFileName(0, tr("Save UAVData File As"), "", filters);
    if (fileName.isEmpty()) {
        return;
    }

    // If the filename ends with .xml, we will do a full export, otherwise, a simple export
    bool fullExport = false;
    if (fileName.endsWith(".xml")) {
        fullExport = true;
    } else if (!fileName.endsWith(".uav")) {
        fileName.append(".uav");
    }

    // generate an XML first (used for all export formats as a formatted data source)
    QString xml = createXMLDocument(Both, fullExport);

    // save file
    QFile file(fileName);
    if (file.open(QIODevice::WriteOnly) &&
            (file.write(xml.toLatin1()) != -1)) {
        file.close();
    } else {
        QMessageBox::critical(0,
                              tr("UAV Data Export"),
                              tr("Unable to save data: ") + fileName,
                              QMessageBox::Ok);
        return;
    }

    QMessageBox msgBox;
    msgBox.setText(tr("Data saved."));
    msgBox.setStandardButtons(QMessageBox::Ok);
    msgBox.exec();
}

/**
 * @brief UAVSettingsImportExportFactory::getUAVSettingsCachePath determines base directory for
 * UAVSetting cache files backup storage.
 * @return returns path to the UAVSetting backup base directory
 */
QString UAVSettingsImportExportFactory::getUAVSettingsCachePath()
{
    // Get CPUSerial (which is a byte array) as a hex string.
    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    UAVObjectUtilManager *utilMngr = pm->getObject<UAVObjectUtilManager>();
    deviceDescriptorStruct board;
    utilMngr->getBoardDescriptionStruct(board);
    QString CPUSerial(utilMngr->getBoardCPUSerial().toHex());

    // Determine UAVSettings cache path
    QString pathName = QDir::cleanPath(Utils::PathUtils().GetStoragePath() + "boardsettingscache"
                                     + QDir::separator() + CPUSerial + QDir::separator());
    return pathName;
}

/**
 * @brief UAVSettingsImportExportFactory::findCache finds the oldest or newest cache file. In a
 * given directory
 * @param pathName directory where cache files are located for a specific board
 * @param what enum that determines whether to return the 'Oldest' or 'Newest' cache file
 * @return filename of the cache found. If no cache is found, null is returned.
 */
QString UAVSettingsImportExportFactory::findCache(QString pathName, const enum which what)
{
    QString fileName;

    QDirIterator it(pathName, QStringList() << "*.uav", QDir::Files, QDirIterator::NoIteratorFlags);
    while (it.hasNext()) {
        QString foundFileName = it.next();

        // UAVSettings cache filenames begin with a timestamp. The most recent cache file is
        // lexicographically first (assuming system time is correct).
        if (fileName.isNull()) {
            fileName = foundFileName;
        } else {
            if (what == Newest && foundFileName > fileName) {
                fileName = foundFileName;
            } else if (what == Oldest && foundFileName < fileName) {
                fileName = foundFileName;
            }
        }
    }
    return fileName;
}
