#ifndef LOGFILE_H
#define LOGFILE_H

#include <QIODevice>
#include <QTime>
#include <QTimer>
#include <QMutexLocker>
#include <QDebug>
#include <QBuffer>
#include <math.h>

#include "uavobjectmanager.h"
#include "utils/xmlconfig.h"
#include "./uavtalk/uavtalk.h"
#include "uavobjects/uavobjectsinit.h"


class LogFile : public QIODevice
{
    Q_OBJECT
public:
    explicit LogFile(QObject *parent = 0);
    qint64 bytesAvailable() const;
    qint64 bytesToWrite() { return logfile.bytesToWrite(); }
    bool open(OpenMode mode);
    void setFileName(QString name) { logfile.setFileName(name); }
    void close();
    qint64 writeData(const char * data, qint64 dataSize);
    qint64 readData(char * data, qint64 maxlen);

    bool startReplay();
    bool stopReplay();

public slots:
    void setReplaySpeed(double val) { playbackSpeed = val; qDebug() << "New playback speed: " << playbackSpeed; }
    void setReplayTime(double val);
    void pauseReplay();
    void resumeReplay();

protected slots:
    void timerFired();
    void uavoUpdated(UAVObject *);

signals:
    void readReady();
    void replayStarted();
    void replayFinished();

protected:
    QByteArray dataBuffer;
    QTimer timer;
    QTime myTime;
    QFile logfile;
    quint32 lastTimeStamp;
    quint32 lastPlayTime;
    QMutex mutex;


    int lastPlayTimeOffset;
    double playbackSpeed;

private:
    QList<quint32> timestampBuffer;
    QList<quint32> timestampPos;
    quint32 timestampBufferIdx;
    quint32 lastTimeStampPos;
    quint32 firstTimestamp;
    quint32 currentTimeStamp;

    UAVObjectManager *xmlUAVObjectManager;
    UAVTalk *xmlTalk;
    QDomDocument doc;
    QDomElement root;
    QDomElement telemetry;
    QDomElement timestamp;
    QDomElement metadata;

    void addUAVObject(UAVDataObject *obj);
    void addInstance(UAVObject *obj, QDomElement *uavobjElement);
    void addArrayField(UAVObjectField *field, QDomElement *uavobjElement);
    void addSingleField(int index, UAVObjectField *field, QDomElement *uavobjElement);
    bool archive(QFile &inputFile, const QString &comment = NULL);
};

#endif // LOGFILE_H
