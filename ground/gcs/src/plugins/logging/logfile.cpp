#include "logfile.h"
#include <QDebug>
#include <QtGlobal>
#include <QTextStream>
#include <QMessageBox>

// autogenerated version info string. MUST GO BEFORE coreconstants.h INCLUDE
#include "../../../../../build/ground/gcs/gcsversioninfo.h"


#include <coreplugin/coreconstants.h>
#include <uavobjectutil/uavobjectutilmanager.h>
#include <extensionsystem/pluginmanager.h>

#include "uavdataobject.h"
#include "uavmetaobject.h"

LogFile::LogFile(QObject *parent) :
    QIODevice(parent),
    timestampBufferIdx(0),
    doc("logfile")
{
    connect(&timer, SIGNAL(timeout()), this, SLOT(timerFired()));
}

/**
 * Opens the logfile QIODevice and the underlying logfile. In case
 * we want to save the logfile, we open in WriteOnly. In case we
 * want to read the logfile, we open in ReadOnly.
 */
bool LogFile::open(OpenMode mode) {

    // start a timer for playback
    myTime.restart();
    if (logfile.isOpen()) {
        // We end up here when doing a replay, because the connection
        // manager will also try to open the QIODevice, even though we just
        // opened it after selecting the file, which happens before the
        // connection manager call...
        return true;
    }

    //Open file as either WriteOnly, or ReadOnly, depending on `mode` parameter
    if(logfile.open(mode) == FALSE)
    {
        qDebug() << "Unable to open " << logfile.fileName() << " for logging";
        return false;
    }

    // TODO: Write a header at the beginng describing objects so that in future
    // they can be read back if ID's change

    /*This addresses part of the TODO. It writes the git hash to the beginning of the file. This will
     * not protect against data losses due to UAVO that have changed when there is no commit to public
     * git, or to commits that are based off of branches that have since been pruned. As such, this
     * can only be seen as a temporary fix.
     */
    if(mode==QIODevice::WriteOnly)
    {
        QString gitHash = QString::fromLatin1(Core::Constants::GCS_REVISION_STR);
        // UAVOSHA1_STR looks something like: "{ 0xbd,0xfc,0x47,0x16,0x59,0xb9,0x08,0x18,0x1c,0x82,0x5e,0x3f,0xe1,0x1a,0x77,0x7f,0x4e,0x06,0xea,0x7c }"
        // This string needs to be reduced to just the hex letters, so in the example we need: bdfc471659b908181c825e3fe11a777f4e06ea7c
        QString uavoHash = QString::fromLatin1(Core::Constants::UAVOSHA1_STR).replace("\"{ ", "").replace(" }\"", "").replace(",", "").replace("0x", "");
        QTextStream out(&logfile);

        out << "Tau Labs git hash:\n" <<  gitHash << "\n" << uavoHash << "\n##\n";

        // Create new UAVObject manager and initialize it with all UAVObjects
        xmlUAVObjectManager = new UAVObjectManager;
        UAVObjectsInitialize(xmlUAVObjectManager);

        // Connect new UAVO manager to a UAVTalk instance
        xmlTalk = new UAVTalk(&logfile, xmlUAVObjectManager);

        /* Create and configure XML document */

        // Reset XML doc
        doc.clear();

        // Start new XML doc
        root = doc.createElement("logfile");
        doc.appendChild(root);

        // add hardware, firmware and GCS version info
        QDomElement versionInfo = doc.createElement("version");
        root.appendChild(versionInfo);
        {
            ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
            UAVObjectUtilManager *utilMngr = pm->getObject<UAVObjectUtilManager>();
            deviceDescriptorStruct board = utilMngr->getBoardDescriptionStruct();

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
        }

        metadata = doc.createElement("metadata");
        root.appendChild(metadata);

        telemetry = doc.createElement("telemetry");
        root.appendChild(telemetry);

        // Connect all UAVOs and seed the XML file. NOTE: Must follow all XML doc creation
        QVector< QVector<UAVDataObject*> > objList = xmlUAVObjectManager->getDataObjects();
        foreach (QVector<UAVDataObject*> list, objList) {
            foreach (UAVDataObject* obj, list) {
                addUAVObject(obj);
            }
        }
    }
    else if(mode == QIODevice::ReadOnly)
    {
        logfile.readLine(); //Read first line of log file. This assumes that the logfile is of the new format.
        QString logGitHashString=logfile.readLine().trimmed(); //Read second line of log file. This assumes that the logfile is of the new format.
        QString logUAVOHashString=logfile.readLine().trimmed(); //Read third line of log file. This assumes that the logfile is of the new format.
        QString gitHash = QString::fromLatin1(Core::Constants::GCS_REVISION_STR);
        QString uavoHash = QString::fromLatin1(Core::Constants::UAVOSHA1_STR).replace("\"{ ", "").replace(" }\"", "").replace(",", "").replace("0x", ""); // See comment above for necessity for string replacements

        if(logUAVOHashString != uavoHash){
            QMessageBox msgBox;
            msgBox.setText("Likely log file incompatibility.");
            msgBox.setInformativeText(QString("The log file was made with branch %1, UAVO hash %2. GCS will attempt to play the file.").arg(logGitHashString).arg(logUAVOHashString));
            msgBox.exec();
        }
        else if(logGitHashString != gitHash){
            QMessageBox msgBox;
            msgBox.setText("Possible log file incompatibility.");
            msgBox.setInformativeText(QString("The log file was made with branch %1. GCS will attempt to play the file.").arg(logGitHashString));
            msgBox.exec();
        }

        QString tmpLine=logfile.readLine(); //Look for the header/body separation string.
        int cnt=0;
        while (tmpLine!="##\n" && cnt < 10 && !logfile.atEnd()){
            tmpLine=logfile.readLine().trimmed();
            cnt++;
        }

        //Check if we reached the end of the file before finding the separation string
        if (cnt >=10 || logfile.atEnd()){
            QMessageBox msgBox;
            msgBox.setText("Corrupted file.");
            msgBox.setInformativeText("GCS cannot find the separation byte. GCS will attempt to play the file."); //<--TODO: add hyperlink to webpage with better description.
            msgBox.exec();

            //Since we could not find the file separator, we need to return to the beginning of the file
            logfile.seek(0);
        }

    }
    else
    {
        qDebug()<< "Logging read/write mode incorrectly set.";
    }

    // Must call parent function for QIODevice to pass calls to writeData
    // We always open ReadWrite, because otherwise we will get tons of warnings
    // during a logfile replay. Read nature is checked upon write ops below.
    QIODevice::open(QIODevice::ReadWrite);

    return true;
}

void LogFile::close()
{
    emit aboutToClose();

    if (timer.isActive())
        timer.stop();
    logfile.close();
    QIODevice::close();

    // Save XML file. Reuse filename and append "xml" to end
    QString xmlLogfileName(logfile.fileName().append(".xml"));

    QString xml = doc.toString(3);
    QFile xmlLogfile(xmlLogfileName);
    if (xmlLogfile.open(QIODevice::WriteOnly) &&
            (xmlLogfile.write(xml.toAscii()) != -1)) {
        xmlLogfile.close();
    } else {
        QMessageBox::critical(0,
                              tr("UAV logfile"),
                              tr("Unable to save log data: ") + xmlLogfileName,
                              QMessageBox::Ok);
        return;
    }

    delete xmlUAVObjectManager;
    delete xmlTalk;
}

qint64 LogFile::writeData(const char * data, qint64 dataSize) {
    if (!logfile.isWritable())
        return dataSize;

    currentTimeStamp = myTime.elapsed();

    logfile.write((char *) &currentTimeStamp,sizeof(currentTimeStamp));
    logfile.write((char *) &dataSize, sizeof(dataSize));

    qint64 written = logfile.write(data, dataSize);
    if(written != -1)
        emit bytesWritten(written);

    // Append last timestamp child before starting new one
    telemetry.appendChild(timestamp);

    // Create
    timestamp = doc.createElement("packet");
    timestamp.setAttribute("Timestamp", currentTimeStamp);


    // Read the data packet from the file.
    QByteArray dataBuffer;
    dataBuffer.append(data, dataSize);

    // Parse the packet. This operation passes the data to the kmlTalk object, which internally parses the data
    // and then emits objectUpdated(UAVObject *) signals. These signals are connected to in the KmlExport constructor.
    for (int i=0; i < dataBuffer.size(); i++) {
        xmlTalk->processInputByte(dataBuffer[i]);
    }

    return dataSize;
}


qint64 LogFile::readData(char * data, qint64 maxSize) {
    QMutexLocker locker(&mutex);
    qint64 toRead = qMin(maxSize,(qint64)dataBuffer.size());
    memcpy(data,dataBuffer.data(),toRead);
    dataBuffer.remove(0,toRead);
    return toRead;
}

qint64 LogFile::bytesAvailable() const
{
    return dataBuffer.size();
}

void LogFile::timerFired()
{
    qint64 dataSize;

    if(logfile.bytesAvailable() > 4)
    {

        int time;
        time = myTime.elapsed();

        //Read packets
        while ((lastPlayTime + ((time - lastPlayTimeOffset)* playbackSpeed) > (lastTimeStamp-firstTimestamp)))
        {
            lastPlayTime += ((time - lastPlayTimeOffset)* playbackSpeed);
            if(logfile.bytesAvailable() < 4) {
                stopReplay();
                return;
            }

            logfile.seek(lastTimeStampPos+sizeof(lastTimeStamp));

            logfile.read((char *) &dataSize, sizeof(dataSize));

            if (dataSize<1 || dataSize>(1024*1024)) {
                qDebug() << "Error: Logfile corrupted! Unlikely packet size: " << dataSize << "\n";
                stopReplay();
                return;
            }
            if(logfile.bytesAvailable() < dataSize) {
                stopReplay();
                return;
            }

            mutex.lock();
            dataBuffer.append(logfile.read(dataSize));
            mutex.unlock();
            emit readyRead();

            if(logfile.bytesAvailable() < 4) {
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

bool LogFile::startReplay() {
    dataBuffer.clear();
    myTime.restart();
    lastPlayTimeOffset = 0;
    lastPlayTime = 0;
    playbackSpeed = 1;

    //Read all log timestamps into array
    timestampBuffer.clear(); //Save beginning of log for later use
    timestampPos.clear();
    quint64 logFileStartIdx = logfile.pos();
    timestampBufferIdx = 0;
    lastTimeStamp = 0;

    while (!logfile.atEnd()){
        qint64 dataSize;

        //Get time stamp position
        timestampPos.append(logfile.pos());

        //Read timestamp and logfile packet size
        logfile.read((char *) &lastTimeStamp, sizeof(lastTimeStamp));
        logfile.read((char *) &dataSize, sizeof(dataSize));

        //Check if dataSize sync bytes are correct.
        //TODO: LIKELY AS NOT, THIS WILL FAIL TO RESYNC BECAUSE THERE IS TOO LITTLE INFORMATION IN THE STRING OF SIX 0x00
        if ((dataSize & 0xFFFFFFFFFFFF0000)!=0){
            qDebug() << "Wrong sync byte. At file location 0x"  << QString("%1").arg(logfile.pos(),0,16) << "Got 0x" << QString("%1").arg(dataSize & 0xFFFFFFFFFFFF0000,0,16) << ", but expected 0x""00"".";
            logfile.seek(timestampPos.last()+1);
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

        logfile.seek(timestampPos.last()+sizeof(lastTimeStamp)+sizeof(dataSize)+dataSize);
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
    logfile.seek(logFileStartIdx+sizeof(lastTimeStamp));
    lastTimeStampPos = timestampPos[0];
    lastTimeStamp = timestampBuffer[0];
    firstTimestamp = timestampBuffer[0];
    timestampBufferIdx = 1;

    timer.setInterval(10);
    timer.start();
    emit replayStarted();
    return true;
}

bool LogFile::stopReplay() {
    close();
    emit replayFinished();
    return true;
}

void LogFile::pauseReplay()
{
    timer.stop();
}

void LogFile::resumeReplay()
{
    lastPlayTimeOffset = myTime.elapsed();
    timer.start();
}

/**
 * @brief LogFile::setReplayTime, sets the playback time
 * @param val, the time in
 */
void LogFile::setReplayTime(double val)
{
    quint32 tmpIdx=0;
    while(timestampBuffer[tmpIdx++] <= val*1000 && tmpIdx <= timestampBufferIdx){
    }

    lastTimeStampPos=timestampPos[tmpIdx];
    lastTimeStamp=timestampBuffer[tmpIdx];
    timestampBufferIdx=tmpIdx;

    lastPlayTimeOffset = myTime.elapsed();
    lastPlayTime=lastTimeStamp;

    qDebug() << "Replaying at: " << lastTimeStamp << ", but requestion at" << val*1000;
}


void LogFile::uavoUpdated(UAVObject *obj)
{
    // add UAVObject to the XML
    QDomElement uavobjElement = doc.createElement("uavobject");
    uavobjElement.setAttribute("Name", obj->getName());
    uavobjElement.setAttribute("UAVObjID", QString("0x") + QString("%1").arg(obj->getObjID(), 8, 16, QChar('0')).toUpper());

    // If the object is a single instance, add it immediately. Otherwise, iterate over list and add all instances
    if (obj->isSingleInstance()) {
        addInstance(obj, &uavobjElement) ;
    } else {
        // Get list of object instances
        QVector<UAVObject*> list = xmlUAVObjectManager->getObjectInstances(obj->getName());
        int index = 0;
        foreach (UAVObject* objInstance, list) {
            // Create a sub-element which contains just this instance
            QDomElement uavobjInstanceElement  = doc.createElement("instance");
            uavobjInstanceElement.setAttribute("value", index);
            addInstance(objInstance, &uavobjInstanceElement) ;

            uavobjElement.appendChild(uavobjInstanceElement);
            index++;
        }

    }

    // append to the timestamp element
    timestamp.appendChild(uavobjElement);
}


/**
 * @brief LogFile::addInstance
 * @param obj UAVObject
 * @param uavobjElement_ptr XML element
 */
void LogFile::addInstance(UAVObject *obj, QDomElement *uavobjElement_ptr)
{
    QList<UAVObjectField*> fields = obj->getFields();
    foreach(UAVObjectField *field, fields) {
        // If an array, drill down. Otherwise, add single field
        if (field->getNumElements() > 1) {
            addArrayField(field, uavobjElement_ptr);
        } else {
            addSingleField(0, field, uavobjElement_ptr);
        }
    }
}


/**
 * @brief LogFile::addArrayField Adds an array field to the XML document
 * @param field UAVObject field
 * @param uavobjElement XML element
 */
void LogFile::addArrayField(UAVObjectField *field, QDomElement *uavobjElement)
{
    QDomElement fieldElement = doc.createElement(field->getName());

    // For each element in the array, add a field
    for (uint i = 0; i < field->getNumElements(); i++) {
        addSingleField(i, field, &fieldElement);
    }
    uavobjElement->appendChild(fieldElement);
}


/**
 * @brief LogFile::addSingleField Adds a single field to the XML document
 * @param index field index
 * @param field UAVObject field
 * @param uavobjElement XML element
 */
void LogFile::addSingleField(int index, UAVObjectField *field, QDomElement *uavobjElement)
{
    QDomElement fieldElement = doc.createElement(field->getName());

    // Add a field element of th appropriate type
    UAVObjectField::FieldType type = field->getType();
    switch (type) {
    case UAVObjectField::BITFIELD:
    case UAVObjectField::ENUM: {
        QStringList options = field->getOptions();
        QVariant value = field->getValue();
        fieldElement.setAttribute("value", options.indexOf(value.toString()));
        fieldElement.setAttribute("units", field->getUnits());
        break;
    }
    case UAVObjectField::INT8:
    case UAVObjectField::INT16:
    case UAVObjectField::INT32:
    case UAVObjectField::UINT8:
    case UAVObjectField::UINT16:
    case UAVObjectField::UINT32:
    case UAVObjectField::FLOAT32:
        fieldElement.setAttribute("value", field->getValue(index).toDouble());
        fieldElement.setAttribute("units", field->getUnits());
        break;
    default:
        Q_ASSERT(false);
    }

    uavobjElement->appendChild(fieldElement);
}



/**
 * @brief LogFile::addUAVObject Adds UAVObject to XML logger
 * @param obj
 */
void LogFile::addUAVObject(UAVDataObject *obj)
{
    // Connect updated signal
    connect(obj, SIGNAL(objectUpdated(UAVObject*)), this, SLOT(uavoUpdated(UAVObject*)));

    // Add metadata to XML
    UAVObject::Metadata mdata = obj->getMetadata();

    QDomElement metadataObject = doc.createElement(obj->getName());
    QDomElement flightUpdateField = doc.createElement("flight_update_period");
    QDomElement gcsUpdateField = doc.createElement("gcs_update_period");
    QDomElement modeFlagsField = doc.createElement("mode_flags");

    flightUpdateField.setAttribute("value", mdata.flightTelemetryUpdatePeriod);
    gcsUpdateField.setAttribute("value", mdata.gcsTelemetryUpdatePeriod);
    modeFlagsField.setAttribute("value", QString("0x")+ QString("%1").arg(mdata.flags, 2, 16, QChar('0')).toUpper());

    // Add fields to metadata object
    metadataObject.appendChild(flightUpdateField);
    metadataObject.appendChild(gcsUpdateField);
    metadataObject.appendChild(modeFlagsField);

    // Add metadata group to metadata root.
    metadata.appendChild(metadataObject);
}
