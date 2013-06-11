/**
 ******************************************************************************
 * @file       kmlexport.cpp
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013.
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @brief Exports log data to KML
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

#include "kmlexport.h"
#include <QDebug>
#include <QtGlobal>
#include <QTextStream>
 #include <QMessageBox>

// autogenerated version info string. MUST GO BEFORE coreconstants.h INCLUDE
#include "../../../../../build/ground/gcs/gcsversioninfo.h"

#include <coreplugin/coreconstants.h>

const double ColorMap_Jet[256][3] = COLORMAP_JET;
#define maxVelocity 20 // This shouldn't be hardcoded

KmlExport::KmlExport(QObject *parent) :
    QIODevice(parent),
    timestampBufferIdx(0)
{
//    exportToKML();
//    connect(&timer, SIGNAL(timeout()), this, SLOT(timerFired()));
}

/**
 * Opens the logfile QIODevice and the underlying logfile. In case
 * we want to save the logfile, we open in WriteOnly. In case we
 * want to read the logfile, we open in ReadOnly.
 */
bool KmlExport::open(OpenMode mode) {

    if (logFile.isOpen()) {
        // We end up here when doing a replay, because the connection
        // manager will also try to open the QIODevice, even though we just
        // opened it after selecting the file, which happens before the
        // connection manager call...
        return true;
    }

    //Open log file as  ReadOnly, depending on `mode` parameter
    if(logFile.open(QIODevice::ReadOnly) == FALSE)
    {
        qDebug() << "Unable to open " << logFile.fileName() << " for logging";
        return false;
    }
    {
        logFile.readLine(); //Read first line of log file. This assumes that the logfile is of the new format.
        QString logGitHashString=logFile.readLine().trimmed(); //Read second line of log file. This assumes that the logfile is of the new format.
        QString logUAVOHashString=logFile.readLine().trimmed(); //Read third line of log file. This assumes that the logfile is of the new format.
        QString gitHash = QString::fromLatin1(Core::Constants::GCS_REVISION_STR);
        QString uavoHash = QString::fromLatin1(Core::Constants::UAVOSHA1_STR).replace("\"{ ", "").replace(" }\"", "").replace(",", "").replace("0x", ""); // See comment above for necessity for string replacements

        if(logUAVOHashString != uavoHash){
            QMessageBox msgBox;
            msgBox.setText("Likely log file incompatibility.");
            msgBox.setInformativeText(QString("The log file was made with branch %1, UAVO hash %2. GCS will attempt to export the file.").arg(logGitHashString).arg(logUAVOHashString));
            msgBox.exec();
        }
        else if(logGitHashString != gitHash){
            QMessageBox msgBox;
            msgBox.setText("Possible log file incompatibility.");
            msgBox.setInformativeText(QString("The log file was made with branch %1. GCS will attempt to export the file.").arg(logGitHashString));
            msgBox.exec();
        }

        QString tmpLine=logFile.readLine(); //Look for the header/body separation string.
        int cnt=0;
        while (tmpLine!="##\n" && cnt < 10 && !logFile.atEnd()){
            tmpLine=logFile.readLine().trimmed();
            cnt++;
        }

        //Check if we reached the end of the file before finding the separation string
        if (cnt >=10 || logFile.atEnd()){
            QMessageBox msgBox;
            msgBox.setText("Corrupted file.");
            msgBox.setInformativeText("GCS cannot find the separation byte. GCS will attempt to export the file."); //<--TODO: add hyperlink to webpage with better description.
            msgBox.exec();

            //Since we could not find the file separator, we need to return to the beginning of the file
            logFile.seek(0);
        }

    }

    // Must call parent function for QIODevice to pass calls to writeData
    // We always open ReadWrite, because otherwise we will get tons of warnings
    // during a logfile replay. Read nature is checked upon write ops below.
    QIODevice::open(QIODevice::ReadWrite); //TODO: Test if this is also the case with KML export

    return true;
}

void KmlExport::close()
{
    emit aboutToClose();

    logFile.close();
    QIODevice::close();
}

qint64 KmlExport::writeData(const char * data, qint64 dataSize) {
    if (!logFile.isWritable())
        return dataSize;

    quint32 timeStamp = myTime.elapsed();

    logFile.write((char *) &timeStamp,sizeof(timeStamp));
    logFile.write((char *) &dataSize, sizeof(dataSize));

    qint64 written = logFile.write(data, dataSize);
    if(written != -1)
        emit bytesWritten(written);

    return dataSize;
}

qint64 KmlExport::readData(char * data, qint64 maxSize) {
    QMutexLocker locker(&mutex);
    qint64 toRead = qMin(maxSize,(qint64)dataBuffer.size());
    memcpy(data,dataBuffer.data(),toRead);
    dataBuffer.remove(0,toRead);
    return toRead;
}

qint64 KmlExport::bytesAvailable() const
{
    return dataBuffer.size();
}

void KmlExport::parseLogFile()
{
    qint64 dataSize;

    if(logFile.bytesAvailable() > 4)
    {

        int time;
        time = myTime.elapsed();

        //Read packets
//        while ((lastPlayTime + ((time - lastPlayTimeOffset)* playbackSpeed) > (lastTimeStamp-firstTimestamp)))
        while (1)
        {
//            lastPlayTime += ((time - lastPlayTimeOffset)* playbackSpeed);
            if(logFile.bytesAvailable() < 4) {
                stopReplay();
                return;
            }

            logFile.seek(lastTimeStampPos+sizeof(lastTimeStamp));

            logFile.read((char *) &dataSize, sizeof(dataSize));

            if (dataSize<1 || dataSize>(1024*1024)) {
                qDebug() << "Error: Logfile corrupted! Unlikely packet size: " << dataSize << "\n";
                QMessageBox::critical(new QWidget(),"Corrupted file", "Incorrect packet size. Stopping export. Data up to this point will be saved.");

//                QMessageBox msgBox;
//                msgBox.setText("Corrupted file.");
//                msgBox.setInformativeText("Incorrect packet size. Stopping export. Data up to this point will be saved");
//                msgBox.exec();

                stopReplay();
                return;
            }
            if(logFile.bytesAvailable() < dataSize) {
                stopReplay();
                return;
            }

            mutex.lock();
            dataBuffer.append(logFile.read(dataSize));
            mutex.unlock();
            emit readyRead();

            if(logFile.bytesAvailable() < 4) {
                stopReplay();
                return;
            }

            lastTimeStampPos = timestampPos[timestampBufferIdx];
            lastTimeStamp = timestampBuffer[timestampBufferIdx];
            timestampBufferIdx++;

            lastPlayTimeOffset = time;
            time = myTime.elapsed();

        }
    } else {
        stopReplay();
    }

}

bool KmlExport::startReplay() {
    dataBuffer.clear();
    myTime.restart();
    lastPlayTimeOffset = 0;
//    lastPlayTime = 0;
//    playbackSpeed = 1;

    //Read all log timestamps into array
    timestampBuffer.clear(); //Save beginning of log for later use
    timestampPos.clear();
    quint64 logFileStartIdx = logFile.pos();
    timestampBufferIdx = 0;
    lastTimeStamp = 0;

    while (!logFile.atEnd()){
        qint64 dataSize;

        //Get time stamp position
        timestampPos.append(logFile.pos());

        //Read timestamp and logfile packet size
        logFile.read((char *) &lastTimeStamp, sizeof(lastTimeStamp));
        logFile.read((char *) &dataSize, sizeof(dataSize));

        //Check if dataSize sync bytes are correct.
        //TODO: LIKELY AS NOT, THIS WILL FAIL TO RESYNC BECAUSE THERE IS TOO LITTLE INFORMATION IN THE STRING OF SIX 0x00
        if ((dataSize & 0xFFFFFFFFFFFF0000)!=0){
            qDebug() << "Wrong sync byte. At file location 0x"  << QString("%1").arg(logFile.pos(),0,16) << "Got 0x" << QString("%1").arg(dataSize & 0xFFFFFFFFFFFF0000,0,16) << ", but expected 0x""00"".";
            logFile.seek(timestampPos.last()+1);
            timestampPos.pop_back();
            continue;
        }

        //Check if timestamps are sequential.
        if (!timestampBuffer.isEmpty() && lastTimeStamp < timestampBuffer.last()){
            QMessageBox msgBox;
            msgBox.setText("Corrupted file.");
            msgBox.setInformativeText("Timestamps are not sequential. Playback may have unexpected behavior"); //<--TODO: add hyperlink to webpage with better description.
            msgBox.exec();

            qDebug() << "Timestamp: " << timestampBuffer.last() << " " << lastTimeStamp;
        }

        timestampBuffer.append(lastTimeStamp);

        logFile.seek(timestampPos.last()+sizeof(lastTimeStamp)+sizeof(dataSize)+dataSize);
    }

    //Check if any timestamps were successfully read
    if (timestampBuffer.size() == 0){
        QMessageBox msgBox;
        msgBox.setText("Empty logfile.");
        msgBox.setInformativeText("No log data can be found.");
        msgBox.exec();

        stopReplay();
        return false;
    }

    //Reset to log beginning.
    logFile.seek(logFileStartIdx+sizeof(lastTimeStamp));
    lastTimeStampPos = timestampPos[0];
    lastTimeStamp = timestampBuffer[0];
    firstTimestamp = timestampBuffer[0];
    timestampBufferIdx = 1;


    // Call parser
    parseLogFile();

    return true;
}

bool KmlExport::stopReplay() {
    close();
    emit replayFinished();
    return true;
}


PlacemarkPtr CreateLineStringPlacemark(const LLAVCoordinates &startPoint, const LLAVCoordinates &endPoint)
{
  KmlFactory* factory = KmlFactory::GetFactory();

  CoordinatesPtr coordinates = factory->CreateCoordinates();
  coordinates->add_latlngalt(startPoint.latitude, startPoint.longitude, startPoint.altitude);
  coordinates->add_latlngalt(endPoint.latitude,   endPoint.longitude,   endPoint.altitude);

  LineStringPtr linestring = factory->CreateLineString();
  linestring->set_extrude(false); // Do not extrude to ground
  linestring->set_tessellate(true);  // Just matching what I did for Ponthy
  linestring->set_altitudemode(kmldom::ALTITUDEMODE_ABSOLUTE);
  linestring->set_coordinates(coordinates);

  LineStylePtr linestyle = factory->CreateLineStyle();
  double currentVelocity = (startPoint.velocity + endPoint.velocity)/2;
  uint8_t colorMapIdx = currentVelocity/maxVelocity * 255;
  uint8_t a = 255;
  uint8_t r = round(ColorMap_Jet[colorMapIdx][0]*255); // M is the jet color map.
  uint8_t g = round(ColorMap_Jet[colorMapIdx][1]*255);
  uint8_t b = round(ColorMap_Jet[colorMapIdx][2]*255);
  linestyle->set_color(kmlbase::Color32(a, b, g, r));

  StylePtr style = factory->CreateStyle();
  style->set_linestyle(linestyle);

  PlacemarkPtr placemark = factory->CreatePlacemark();
  placemark->set_geometry(linestring);
  placemark->set_name("tl_plot");
  placemark->set_style(style);
  placemark->set_visibility(true);

  return placemark;
}

// We assume that we already have the necessary inputs parsed. This lays the groundwork for exporting it
void KmlExport::exportToKML(const QString &outputFileName)
{

    LLAVCoordinates endPoint;
    LLAVCoordinates startPoint;

    startPoint.altitude = 100;
    startPoint.latitude = 49;
    startPoint.longitude = 49;
    startPoint.velocity = 5;

    endPoint.altitude = 100;
    endPoint.latitude = 49;
    endPoint.longitude = 49;
    endPoint.velocity = 15;

    PlacemarkPtr bob = CreateLineStringPlacemark(startPoint, endPoint);
    PlacemarkPtr bob2 = CreateLineStringPlacemark(startPoint, endPoint);

    // Get the factory singleton to create KML elements.
    KmlFactory* factory = KmlFactory::GetFactory();

    // Create <Document> and give it features
    DocumentPtr document = factory->CreateDocument();
    document->add_feature(bob);
    document->add_feature(bob2);

    // Create <kml> and give it <Document>.
    KmlPtr kml = factory->CreateKml();
    kml->set_feature(document);  // kml takes ownership.

    // Serialize to XML
    std::string kml_data = kmldom::SerializePretty(kml);

    // Print to stdout
    std::cout << kml_data;

    // Save to file
    if ((outputFileName== NULL) || !kmlbase::File::WriteStringToFile(kml_data, outputFileName.toStdString())) {
        qDebug() << "KML write failed: " << outputFileName;
//        return false;
    }
//    return true;
}
