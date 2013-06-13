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
#include <QMessageBox>
#include <QTextStream>
#include <QtGlobal>

#include <coreplugin/coreconstants.h>
#include "utils/coordinateconversions.h"
#include "uavobjects/uavobjectsinit.h"

// autogenerated version info string. MUST GO BEFORE coreconstants.h INCLUDE
#include "../../../../../build/ground/gcs/gcsversioninfo.h"


const double ColorMap_Jet[256][3] = COLORMAP_JET;
#define maxVelocity 20 // This shouldn't be hardcoded

KmlExport::KmlExport(QString inputLogFileName, QString outputKmlFileName) :
    outputFileName(outputKmlFileName)
{
    // Create new UAVObject manager and initialize it with all UAVObjects
    UAVObjectManager *kmlParser = new UAVObjectManager;
    UAVObjectsInitialize(kmlParser);

    connect(kmlParser->getObject("PositionActual"), SIGNAL(objectUpdated(UAVObject *)), this, SLOT(uavobjectUpdated(UAVObject *)));

    kmlTalk = new UAVTalk(&logFile, kmlParser);

    logFile.setFileName(inputLogFileName);

    // Get the UAVObjects
    homeLocation = HomeLocation::GetInstance(kmlParser);
    positionActual = PositionActual::GetInstance(kmlParser);
    velocityActual = VelocityActual::GetInstance(kmlParser);

    // Get the factory singleton to create KML elements.
    factory = KmlFactory::GetFactory();

    // Create <Document>
    document = factory->CreateDocument();
}

/**
 * Opens the logfile QIODevice and the underlying logfile. In case
 * we want to save the logfile, we open in WriteOnly. In case we
 * want to read the logfile, we open in ReadOnly.
 */
bool KmlExport::open()
{
    if (logFile.isOpen()) {
        logFile.close();
    }

    //Open log file as  ReadOnly
    if(logFile.open(QIODevice::ReadOnly) == FALSE)
    {
        qDebug() << "Unable to open " << logFile.fileName() << " for logging";
        return false;
    }

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

    return true;
}

void KmlExport::parseLogFile()
{
    qint64 packetSize;
    quint32 timeStampIdx;

    //Read packets
    while (!logFile.atEnd())
    {
        if(logFile.bytesAvailable() < 4) {
            break;
        }

        //Read timestamp and logfile packet size
        logFile.read((char *) &timeStamp, sizeof(timeStamp));
        logFile.read((char *) &packetSize, sizeof(packetSize));

        if (packetSize<1 || packetSize>(1024*1024)) {
            qDebug() << "Error: Logfile corrupted! Unlikely packet size: " << packetSize << "\n";
            QMessageBox::critical(new QWidget(),"Corrupted file", "Incorrect packet size. Stopping export. Data up to this point will be saved.");

            break;
        }

        if(logFile.bytesAvailable() < packetSize) {
            break;
        }

        // Read the data packet from the file.
        QByteArray dataBuffer;
        dataBuffer.append(logFile.read(packetSize));

        // Parse the packet. This operation passes the data to the kmlTalk object, which internally parses the data
        // and then emits objectUpdated(UAVObject *) signals. These signals are connected to in the KmlExport constructor.
        for (int i=0; i < dataBuffer.size(); i++) {
            kmlTalk->processInputByte(dataBuffer[i]);
        }

        timeStampIdx++;
    }

    stopReplay();
}

bool KmlExport::startReplay()
{
    //Read all log timestamps into array
    timestampBuffer.clear(); //Save beginning of log for later use
    timestampPos.clear();
    quint64 logFileStartIdx = logFile.pos();
    quint32 lastTimeStamp = 0;

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

    //Reset to log beginning, including the timestamp.
    logFile.seek(logFileStartIdx);

    // Call parser
    parseLogFile();

    return true;
}

bool KmlExport::stopReplay() {
    logFile.close();
//    emit replayFinished();
    return true;
}


PlacemarkPtr KmlExport::CreateLineStringPlacemark(const LLAVCoordinates &startPoint, const LLAVCoordinates &endPoint)
{
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

//<name>338.8</name>
//<TimeSpan>
//    <begin>2010-03-26T19:03:11Z</begin>
//    <end>2010-03-26T19:03:11Z</end>
//</TimeSpan>
//<visibility>1</visibility>
//<description><![CDATA[<TABLE border="1"><TR><TD><B>Variable</B></TD><TD><B>Value</B></TD></TR><TR><TD>longitude [decimal degrees]</TD><TD>6.11968</TD></TR><TR><TD>latitude [decimal degrees]</TD><TD>49.6186</TD></TR><TR><TD>elevation [m]</TD><TD>1684.1</TD></TR><TR><TD>tStart</TD><TD>2010-03-26T19:03:11Z</TD></TR><TR><TD>tStop</TD><TD>2010-03-26T19:03:11Z</TD></TR></TABLE>]]>
//</description>
//<IconStyle>
//    <color>FFFFFFFF</color>
//    <scale>1</scale>
//    <Icon>
//        <href>http://earth.google.com/images/kml-icons/track-directional/track-none.png</href>
//    </Icon>
//</IconStyle>
//<styleUrl>#track</styleUrl>
//<Point id="point_point">
//    <altitudeMode>clampToGround</altitudeMode>
//    <tessellate>1</tessellate>
//    <extrude>0</extrude>
//    <coordinates>6.119678,49.618573,1684.1</coordinates>
//</Point>


PlacemarkPtr KmlExport::createTimestampPlacemark(const LLAVCoordinates &timestampPoint, quint32 lastPlacemarkTime, quint32 newPlacemarkTime)
{
    CoordinatesPtr coordinates = factory->CreateCoordinates();
    coordinates->add_latlngalt(timestampPoint.latitude, timestampPoint.longitude, timestampPoint.altitude);

    PointPtr point = factory->CreatePoint();
    point->set_extrude(true); // Extrude to ground
    point->set_altitudemode(kmldom::ALTITUDEMODE_ABSOLUTE);
    point->set_coordinates(coordinates);

    IconStyleIconPtr iconStyleIcon = factory->CreateIconStyleIcon();
    iconStyleIcon->set_href("http://earth.google.com/images/kml-icons/track-directional/track-none.png");

    IconStylePtr iconStyle = factory->CreateIconStyle();
    iconStyle->set_color(kmlbase::Color32(255, 255, 255, 255));
    iconStyle->set_icon(iconStyleIcon);

    StylePtr style = factory->CreateStyle();
    style->set_iconstyle(iconStyle);

    TimeSpanPtr timeSpan = factory->CreateTimeSpan();
    QDateTime startTime = QDateTime::currentDateTimeUtc().addMSecs(lastPlacemarkTime);
    QDateTime endTime = QDateTime::currentDateTimeUtc().addMSecs(newPlacemarkTime);
    QString dateTimeFormat("yyyy-MM-ddThh:mm:ssZ"); // XML Schema time format
    timeSpan->set_begin(startTime.toString(dateTimeFormat).toStdString());
    timeSpan->set_end(endTime.toString(dateTimeFormat).toStdString());

    PlacemarkPtr placemark = factory->CreatePlacemark();
    placemark->set_geometry(point);
    placemark->set_description(QString("Latitude: %1 deg\nLongitude: %2 deg\nAltitude: %3 m\nAirspeed: %4 m/s\nGroundspeed: %5 m/s\n").arg(timestampPoint.latitude).arg(timestampPoint.longitude).arg(timestampPoint.altitude).arg(-1).arg(timestampPoint.velocity).toStdString());
    placemark->set_style(style);
    placemark->set_timeprimitive(timeSpan);
    placemark->set_name(QString("%1").arg(timeStamp / 1000.0).toStdString());
    placemark->set_visibility(true);

    return placemark;
}

// We assume that we already have the necessary inputs parsed. This lays the groundwork for exporting it
void KmlExport::exportToKML()
{
    bool ret = open();
    if (!ret) {
        qDebug () << "Logfile failed to open during KML export";
        return;
    }
    startReplay();

    // Create <kml> and give it <Document>.
    KmlPtr kml = factory->CreateKml();
    kml->set_feature(document);  // kml takes ownership.

    // Serialize to XML
    std::string kml_data = kmldom::SerializePretty(kml);

    // Print to stdout
    std::cout << kml_data;

    // Save to file
    if ((outputFileName == NULL) || !kmlbase::File::WriteStringToFile(kml_data, outputFileName.toStdString())) {
        qDebug() << "KML write failed: " << outputFileName;
    }
}

void KmlExport::uavobjectUpdated(UAVObject *obj)
{
    Q_UNUSED(obj);

    HomeLocation::DataFields homeLocationData = homeLocation->getData();
    PositionActual::DataFields positionActualData = positionActual->getData();
    VelocityActual::DataFields velocityActualData = velocityActual->getData();

    LLAVCoordinates newPoint;

    // Convert NED data to LLA data
    double homeLLA[3]={homeLocationData.Latitude/1e7, homeLocationData.Longitude/1e7, homeLocationData.Altitude};
    double NED[3]={positionActualData.North, positionActualData.East, positionActualData.Down};
    double LLA[3];
    Utils::CoordinateConversions().NED2LLA_HomeLLA(homeLLA, NED, LLA);

    // Generate new placemark
    newPoint.latitude = LLA[0];
    newPoint.longitude = LLA[1];
    newPoint.altitude = LLA[2];
    newPoint.velocity = sqrt(velocityActualData.North*velocityActualData.North + velocityActualData.East*velocityActualData.East);

    PlacemarkPtr newPlacemark = CreateLineStringPlacemark(oldPoint, newPoint);

    // Add the placemark to the KML document
    document->add_feature(newPlacemark);

    // Every 2 seconds generate a time stamp
    if (timeStamp - lastPlacemarkTime > 2000) {

        PlacemarkPtr newPlacemarkTimestamp = createTimestampPlacemark(newPoint, lastPlacemarkTime, timeStamp);
        document->add_feature(newPlacemarkTimestamp);
        lastPlacemarkTime = timeStamp;
    }



    // Copy newPoint to oldPoint
    oldPoint.latitude = newPoint.latitude;
    oldPoint.longitude = newPoint.longitude;
    oldPoint.altitude = newPoint.altitude;
    oldPoint.velocity = newPoint.velocity;

}

