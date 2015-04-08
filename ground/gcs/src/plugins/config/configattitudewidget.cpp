/**
 ******************************************************************************
 *
 * @file       configattitudewidget.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup ConfigPlugin Config Plugin
 * @{
 * @brief The Configuration Gadget used to update settings in the firmware
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
#include "configattitudewidget.h"
#include "physical_constants.h"

#include "math.h"
#include <QDebug>
#include <QTimer>
#include <QStringList>
#include <QWidget>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QPushButton>
#include <QMessageBox>
#include <QThread>
#include <QErrorMessage>
#include <iostream>
#include <QDesktopServices>
#include <QUrl>
#include <coreplugin/iboardtype.h>
#include <attitudesettings.h>
#include <sensorsettings.h>
#include <inssettings.h>
#include <homelocation.h>
#include <accels.h>
#include <gyros.h>
#include <magnetometer.h>
#include <baroaltitude.h>

#include "assertions.h"
#include "calibration.h"

#include <qwt/src/qwt_symbol.h>
#include <qwt/src/qwt_legend.h>
#include <qwt/src/qwt_color_map.h>
#include <qwt/src/qwt_raster_data.h>
#include <qwt/src/qwt_scale_engine.h>

#include <qwtpolar/src/qwt_polar_renderer.h>
#include <qwtpolar/src/qwt_polar_grid.h>
#include <qwtpolar/src/qwt_polar_marker.h>
#include <qwtpolar/src/qwt_polar_spectrogram.h>



#define sign(x) ((x < 0) ? -1 : 1)

// Uncomment this to enable 6 point calibration on the accels
#define SIX_POINT_CAL_ACCEL

const double ConfigAttitudeWidget::maxVarValue = 0.1;

// *****************

class HemisphereSpectrogramData: public QwtRasterData
{
public:
    HemisphereSpectrogramData(bool isLowerHemisphere)
    {
        this->isLowerHemisphere = isLowerHemisphere;
        setInterval(Qt::ZAxis, QwtInterval(0.0, 1.0));
    }

    virtual double value( double azimuth, double radius ) const
    {
        // If there is no sectorPercent data, show default color
        if (sectorPercent.empty() == true) {
            return 0;
        }

        // Determine number of sectors
        uint16_t numRadialSectors = sectorPercent.length()/2;
        uint16_t numAzimulthalSectors = sectorPercent[0].length();

        // Determine in which sector the point falls.
        uint16_t radiusIdx = floor(radius / 180.0 * numRadialSectors);
        uint16_t azimuthIdx = floor(azimuth / 360.0 * numAzimulthalSectors);

        // Saturate the index
        if (azimuthIdx >= numAzimulthalSectors)
            azimuthIdx = numAzimulthalSectors - 1;
        if (radiusIdx >= numRadialSectors)
            radiusIdx = numRadialSectors - 1;

        double category = floor(sectorPercent[radiusIdx + numRadialSectors*isLowerHemisphere][azimuthIdx]*3)/3;

        return category;
    }

    QVector< QVector<float> > sectorPercent;

private:
    bool isLowerHemisphere;
};

/**
 * @brief The ColorMap class Defines a program-wide colormap
 */
class ColorMap: public QwtLinearColorMap
{
public:
    ColorMap():
        QwtLinearColorMap()
    {
        // Set transparency
        uint8_t alpha = 35;

        // The color interval must be created separately. These are the absolute
        // maximum and minimum values of the colormap. The colorstops define the
        // range in between.
        setColorInterval(QColor(255, 255, 255, 0), QColor(0, 255, 0, alpha));

        // Input values given normalized to 1.
        addColorStop( 0.001, QColor(255,   0, 0, alpha));
        addColorStop( 0.500, QColor(255, 255, 0, alpha));
    }
};


/**
 * @brief The JetColorMap class Colormap based off of MATLAB's jet colormap
 */
class JetColorMap
{
public:
    static uint8_t red(double gray) {
        if (gray < -1)
            gray = -1;
        else if (gray > 1)
            gray = 1;
        return round(base(gray - 0.5)*255);
    }
    static uint8_t green(double gray) {
        if (gray < -1)
            gray = -1;
        else if (gray > 1)
            gray = 1;
        return round(base(gray)*255);
    }
    static uint8_t blue(double gray) {
        if (gray < -1)
            gray = -1;
        else if (gray > 1)
            gray = 1;
        return round(base(gray + 0.5)*255);
    }

private:
    static double interpolate(double val, double x0, double y0, double x1, double y1) {
        return (val-x0)*(y1-y0)/(x1-x0) + y0;
    }

    static double base(double val) {
        if (val <= -0.75) return 0;
        else if (val <= -0.25) return interpolate(val, -0.75, 0.0, -0.25, 1.0);
        else if (val <= 0.25) return 1.0;
        else if (val <= 0.75) return interpolate(val, 1.0, 0.25, 0.0, 0.75);
        else return 0.0;
    }
};


class Thread : public QThread
{
public:
    static void usleep(unsigned long usecs)
    {
        QThread::usleep(usecs);
    }
};

// *****************

ConfigAttitudeWidget::ConfigAttitudeWidget(QWidget *parent) :
    ConfigTaskWidget(parent),
    m_ui(new Ui_AttitudeWidget()),
    board_has_accelerometer(false),
    board_has_magnetometer(false)
{
    m_ui->setupUi(this);

    // Initialization of the ellipsoid display widget
    seuptEllipsoidDisplay();

    // Initialization of the Paper plane widget
    m_ui->sixPointHelp->setScene(new QGraphicsScene(this));

    paperplane = new QGraphicsSvgItem();
    paperplane->setSharedRenderer(new QSvgRenderer());
    paperplane->renderer()->load(QString(":/configgadget/images/paper-plane.svg"));
    paperplane->setElementId("plane-horizontal");
    m_ui->sixPointHelp->scene()->addItem(paperplane);
    m_ui->sixPointHelp->setSceneRect(paperplane->boundingRect());

    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    UAVObjectUtilManager* utilMngr = pm->getObject<UAVObjectUtilManager>();
    Q_ASSERT(utilMngr);
    if (utilMngr != NULL) {
        Core::IBoardType *board = utilMngr->getBoardType();
        if (board != NULL) {
            board_has_accelerometer = board->queryCapabilities(Core::IBoardType::BOARD_CAPABILITIES_ACCELS);
            board_has_magnetometer = board->queryCapabilities(Core::IBoardType::BOARD_CAPABILITIES_MAGS);
        }
    }

    // Must set up the UI (above) before setting up the UAVO mappings or refreshWidgetValues
    // will be dealing with some null pointers
    addUAVObject("AttitudeSettings");
    addUAVObject("SensorSettings");
    if (board_has_magnetometer) {
        addUAVObject("INSSettings");
    }
    autoLoadWidgets();

    // Configure the calibration object
    calibration.initialize(board_has_accelerometer, board_has_magnetometer);

    // Configure the calibration UI
    m_ui->cbCalibrateAccels->setEnabled(board_has_accelerometer);
    m_ui->cbCalibrateMags->setEnabled(board_has_magnetometer);
    if (board_has_accelerometer) {
        m_ui->cbCalibrateAccels->setChecked(board_has_accelerometer);
    } else if (board_has_magnetometer) {
        m_ui->cbCalibrateMags->setChecked(board_has_magnetometer);
    }

    // Must connect the graphs to the calibration object to see the calibration results
    calibration.configureTempCurves(m_ui->xGyroTemp, m_ui->yGyroTemp, m_ui->zGyroTemp);

    // Connect the signals
    connect(m_ui->yawOrientationStart, SIGNAL(clicked()), &calibration, SLOT(doStartOrientation()));
    connect(m_ui->levelingStart, SIGNAL(clicked()), &calibration, SLOT(doStartNoBiasLeveling()));
    connect(m_ui->levelingAndBiasStart, SIGNAL(clicked()), &calibration, SLOT(doStartBiasAndLeveling()));
    connect(m_ui->sixPointStart, SIGNAL(clicked()), &calibration, SLOT(doStartSixPoint()));
    connect(m_ui->sixPointSave, SIGNAL(clicked()), &calibration, SLOT(doSaveSixPointPosition()));
    connect(m_ui->sixPointCancel, SIGNAL(clicked()), &calibration, SLOT(doCancelSixPoint()));
    connect(m_ui->pb_ellipsoidCalibration, SIGNAL(clicked()), &calibration, SLOT(doEllipsoidCalibration()));
    connect(m_ui->pb_ellipsoidCalibrationCancel, SIGNAL(clicked()), &calibration, SLOT(doCancelEllipsoidCalibration()));
    connect(m_ui->cbCalibrateAccels, SIGNAL(clicked()), this, SLOT(configureSixPoint()));
    connect(m_ui->cbCalibrateMags, SIGNAL(clicked()), this, SLOT(configureSixPoint()));
    connect(m_ui->startTempCal, SIGNAL(clicked()), &calibration, SLOT(doStartTempCal()));
    connect(m_ui->acceptTempCal, SIGNAL(clicked()), &calibration, SLOT(doAcceptTempCal()));
    connect(m_ui->cancelTempCal, SIGNAL(clicked()), &calibration, SLOT(doCancelTempCalPoint()));
    connect(m_ui->tempCalRange, SIGNAL(valueChanged(int)), &calibration, SLOT(setTempCalRange(int)));
    calibration.setTempCalRange(m_ui->tempCalRange->value());

    // Let calibration update the UI
    connect(&calibration, SIGNAL(yawOrientationProgressChanged(int)), m_ui->pb_yawCalibration, SLOT(setValue(int)));
    connect(&calibration, SIGNAL(levelingProgressChanged(int)), m_ui->accelBiasProgress, SLOT(setValue(int)));
    connect(&calibration, SIGNAL(tempCalProgressChanged(int)), m_ui->tempCalProgress, SLOT(setValue(int)));
    connect(&calibration, SIGNAL(showTempCalMessage(QString)), m_ui->tempCalMessage, SLOT(setText(QString)));
    connect(&calibration, SIGNAL(sixPointProgressChanged(int)), m_ui->sixPointProgress, SLOT(setValue(int)));
    connect(&calibration, SIGNAL(showSixPointMessage(QString)), m_ui->sixPointCalibInstructions, SLOT(setText(QString)));
    connect(&calibration, SIGNAL(updatePlane(int)), this, SLOT(displayPlane(int)));
    connect(&calibration, SIGNAL(updateEllipsoidFit(QVector<QVector<double> >, QVector< QVector<float> >, bool)), this, SLOT(displayEllipsoidFit(QVector<QVector<double> >, QVector< QVector<float> >, bool)));

    // Let the calibration gadget control some control enables
    connect(&calibration, SIGNAL(toggleSavePosition(bool)), m_ui->sixPointSave, SLOT(setEnabled(bool)));
    connect(&calibration, SIGNAL(toggleControls(bool)), m_ui->sixPointStart, SLOT(setEnabled(bool)));
    connect(&calibration, SIGNAL(toggleControls(bool)), m_ui->sixPointCancel, SLOT(setDisabled(bool)));
    connect(&calibration, SIGNAL(toggleControls(bool)), m_ui->pb_ellipsoidCalibration, SLOT(setEnabled(bool)));
    connect(&calibration, SIGNAL(toggleControls(bool)), m_ui->yawOrientationStart, SLOT(setEnabled(bool)));
    connect(&calibration, SIGNAL(toggleControls(bool)), m_ui->levelingStart, SLOT(setEnabled(bool)));
    connect(&calibration, SIGNAL(toggleControls(bool)), m_ui->levelingAndBiasStart, SLOT(setEnabled(bool)));
    connect(&calibration, SIGNAL(toggleControls(bool)), m_ui->startTempCal, SLOT(setEnabled(bool)));
    connect(&calibration, SIGNAL(toggleControls(bool)), m_ui->acceptTempCal, SLOT(setDisabled(bool)));
    connect(&calibration, SIGNAL(toggleControls(bool)), m_ui->cancelTempCal, SLOT(setDisabled(bool)));

    // Let the calibration gadget mark the tab as dirty, i.e. having unsaved data.
    connect(&calibration, SIGNAL(calibrationCompleted()), this, SLOT(do_SetDirty()));

    // Let the calibration class mark the widget as busy
    connect(&calibration, SIGNAL(calibrationBusy(bool)), this, SLOT(onCalibrationBusy(bool)));

    m_ui->sixPointStart->setEnabled(true);
    m_ui->yawOrientationStart->setEnabled(true);
    m_ui->levelingStart->setEnabled(true);
    m_ui->levelingAndBiasStart->setEnabled(true);

    // F1 boards don't have this object
    setNotMandatory("StateEstimation");

    refreshWidgetsValues();
}

ConfigAttitudeWidget::~ConfigAttitudeWidget()
{
    // Do nothing
}


void ConfigAttitudeWidget::showEvent(QShowEvent *event)
{
    Q_UNUSED(event)
    m_ui->sixPointHelp->fitInView(paperplane,Qt::KeepAspectRatio);
}

void ConfigAttitudeWidget::resizeEvent(QResizeEvent *event)
{
    Q_UNUSED(event)
    m_ui->sixPointHelp->fitInView(paperplane,Qt::KeepAspectRatio);
}


/**
 * @brief ConfigAttitudeWidget::seuptEllipsoidDisplay
 */
void ConfigAttitudeWidget::seuptEllipsoidDisplay()
{
    /* Scatterpoint graph*/
    upperHemisphere_plot = new QwtPolarPlot(QwtText("Upper hemisphere"), this);
    lowerHemisphere_plot = new QwtPolarPlot(QwtText("Lower hemisphere"), this);

    layout = new QHBoxLayout(m_ui->sixPointCalibInstructions);
    layout->addWidget(upperHemisphere_plot, 10);
    layout->addWidget(lowerHemisphere_plot, 10);

    const QwtInterval radialInterval(0.0, 180.0);
    const QwtInterval azimuthInterval(0.0, 360.0);

    // Configure polar plots
    upperHemisphere_plot->setAutoReplot(false);
    upperHemisphere_plot->setPlotBackground(Qt::white);
    lowerHemisphere_plot->setAutoReplot(false);
    lowerHemisphere_plot->setPlotBackground(Qt::white);

    // Configure origin angles
    upperHemisphere_plot->setAzimuthOrigin(M_PI/2.0);
    lowerHemisphere_plot->setAzimuthOrigin(3*M_PI/2.0);

    // Configure angle scales
    upperHemisphere_plot->setScaleMaxMinor(QwtPolar::Azimuth, 1);
    upperHemisphere_plot->setScale(QwtPolar::Azimuth,
        azimuthInterval.maxValue(), azimuthInterval.minValue(),
        azimuthInterval.width() / 12);
    upperHemisphere_plot->setScaleMaxMinor(QwtPolar::Azimuth, 1);
    lowerHemisphere_plot->setScale(QwtPolar::Azimuth,
        azimuthInterval.minValue(), azimuthInterval.maxValue(),
        azimuthInterval.width() / 12);

    // Configure radial scales
    upperHemisphere_plot->setScaleMaxMinor(QwtPolar::Radius, 2);
    upperHemisphere_plot->setScaleMaxMajor(QwtPolar::Radius, 3);
    upperHemisphere_plot->setScale(QwtPolar::Radius,
        radialInterval.minValue(), radialInterval.maxValue());

    lowerHemisphere_plot->setScaleMaxMinor(QwtPolar::Radius, 2);
    lowerHemisphere_plot->setScaleMaxMajor(QwtPolar::Radius, 3);
    lowerHemisphere_plot->setScale(QwtPolar::Radius,
        radialInterval.minValue(), radialInterval.maxValue());

    // Configure grids, axes
    upperHemisphere_grid = new QwtPolarGrid();
    lowerHemisphere_grid = new QwtPolarGrid();
    upperHemisphere_grid->setPen(QPen(Qt::black));
    lowerHemisphere_grid->setPen(QPen(Qt::black));
    for (int scaleId = 0; scaleId < QwtPolar::ScaleCount; scaleId++)
    {
        upperHemisphere_grid->showGrid(scaleId);
        upperHemisphere_grid->showMinorGrid(scaleId);
        lowerHemisphere_grid->showGrid(scaleId);
        lowerHemisphere_grid->showMinorGrid(scaleId);

        QPen minorPen(Qt::gray);
        upperHemisphere_grid->setMinorGridPen(scaleId, minorPen);
        lowerHemisphere_grid->setMinorGridPen(scaleId, minorPen);
    }
    upperHemisphere_grid->setAxisPen(QwtPolar::AxisAzimuth, QPen(Qt::black));
    lowerHemisphere_grid->setAxisPen(QwtPolar::AxisAzimuth, QPen(Qt::black));

    upperHemisphere_grid->showAxis(QwtPolar::AxisAzimuth, true);
    upperHemisphere_grid->showAxis(QwtPolar::AxisLeft, false);
    upperHemisphere_grid->showAxis(QwtPolar::AxisRight, true);
    upperHemisphere_grid->showAxis(QwtPolar::AxisTop, true);
    upperHemisphere_grid->showAxis(QwtPolar::AxisBottom, false);
    upperHemisphere_grid->showGrid(QwtPolar::Azimuth, true);
    upperHemisphere_grid->showGrid(QwtPolar::Radius, true);
    lowerHemisphere_grid->showAxis(QwtPolar::AxisAzimuth, true);
    lowerHemisphere_grid->showAxis(QwtPolar::AxisLeft, false);
    lowerHemisphere_grid->showAxis(QwtPolar::AxisRight, true);
    lowerHemisphere_grid->showAxis(QwtPolar::AxisTop, true);
    lowerHemisphere_grid->showAxis(QwtPolar::AxisBottom, false);
    lowerHemisphere_grid->showGrid(QwtPolar::Azimuth, true);
    lowerHemisphere_grid->showGrid(QwtPolar::Radius, true);

    upperHemisphere_grid->attach(upperHemisphere_plot);
    lowerHemisphere_grid->attach(lowerHemisphere_plot);

    /* Raster graph. This provides the background shading. */
    upperHemisphere_spectrogram = new QwtPolarSpectrogram();
    lowerHemisphere_spectrogram = new QwtPolarSpectrogram();
    upperHemisphere_spectrogramData = new HemisphereSpectrogramData(false);
    lowerHemisphere_spectrogramData = new HemisphereSpectrogramData(true);
    upperHemisphere_spectrogram->setPaintAttribute(QwtPolarSpectrogram::ApproximatedAtan, true);
    lowerHemisphere_spectrogram->setPaintAttribute(QwtPolarSpectrogram::ApproximatedAtan, true);
    upperHemisphere_spectrogram->setRenderThreadCount(0); // use multi threading
    lowerHemisphere_spectrogram->setRenderThreadCount(0); // use multi threading
    upperHemisphere_spectrogram->setData(upperHemisphere_spectrogramData);
    lowerHemisphere_spectrogram->setData(lowerHemisphere_spectrogramData);
    upperHemisphere_spectrogram->setColorMap(new ColorMap());
    lowerHemisphere_spectrogram->setColorMap(new ColorMap());

    upperHemisphere_spectrogram->attach(upperHemisphere_plot);
    lowerHemisphere_spectrogram->attach(lowerHemisphere_plot);
}

/**
 * @brief ConfigAttitudeWidget::displayEllipsoidFit Plot the point cloud
 * @param data n x 3 matrix of points
 */
void ConfigAttitudeWidget::displayEllipsoidFit(QVector< QVector<double> > data, QVector< QVector<float> > sectorPercent, bool lastRun)
{
    /* Scatterpoint */
    // Clear old marker points
    foreach (QwtPolarMarker *marker, markerList) {
        marker->detach();
    }
    markerList.clear();

    // Plot scatter points. The color represents the deviation from the unit sphere
    foreach (QVector<double> point, data) {
        const double a_D = atan2(point[1], point[0]) * RAD2DEG;
        const double r = sqrt(point[0]*point[0] + point[1]*point[1]) * 180;
        const double err = sqrt(point[0]*point[0] + point[1]*point[1] + point[2]*point[2]) - 1.0;
        const double errScaleFactor = 10; // Scale by 10 in order to magnify error

        const uint8_t alpha = 100; // Set transparency
        const QColor color(JetColorMap::red(errScaleFactor*err),
                           JetColorMap::green(errScaleFactor*err),
                           JetColorMap::blue(errScaleFactor*err), alpha);

        QwtPolarMarker *marker = new QwtPolarMarker();
        marker->setPosition(QwtPointPolar(a_D, r));
        marker->setSymbol(new QwtSymbol(QwtSymbol::Ellipse, QBrush(color), QPen(color), QSize(4, 4)));
        marker->setLabelAlignment(Qt::AlignHCenter | Qt::AlignTop);

        // If point is on or above the plane, plot in the upper hemisphere. Otherwise, plot in the lower hemisphere.
        if (point[2] >= 0)
            marker->attach(upperHemisphere_plot);
        else
            marker->attach(lowerHemisphere_plot);

        markerList.append(marker);
    }

    // Replot last point as a large X marker with a bright color, making it clear where the magnetometer is currently pointed
    markerList.last()->setSymbol(new QwtSymbol(QwtSymbol::XCross, QBrush(Qt::magenta), QPen(Qt::magenta), QSize(14, 14)));

    /* Raster map */
    if (!(data.length() % 20) || lastRun == true) {
        upperHemisphere_spectrogramData->sectorPercent = sectorPercent;
        lowerHemisphere_spectrogramData->sectorPercent = sectorPercent;
    }

    // Trigger graph redraw
    upperHemisphere_plot->replot();
    lowerHemisphere_plot->replot();
}


/**
  Rotate the paper plane
  */
void ConfigAttitudeWidget::displayPlane(int position)
{
    QString displayElement;
    switch(position) {
    case 1:
        displayElement = "plane-horizontal";
        break;
    case 2:
        displayElement = "plane-left";
        break;
    case 3:
        displayElement = "plane-flip";
        break;
    case 4:
        displayElement = "plane-right";
        break;
    case 5:
        displayElement = "plane-up";
        break;
    case 6:
        displayElement = "plane-down";
        break;
    default:
        return;
    }

    paperplane->setElementId(displayElement);
    m_ui->sixPointHelp->setSceneRect(paperplane->boundingRect());
    m_ui->sixPointHelp->fitInView(paperplane,Qt::KeepAspectRatio);
}

/********** UI Functions *************/

/**
  * Called by the ConfigTaskWidget parent when variances are updated
  * to update the UI
  */
void ConfigAttitudeWidget::refreshWidgetsValues(UAVObject *)
{
    ConfigTaskWidget::refreshWidgetsValues();
}

/**
 * @brief ConfigAttitudeWidget::setUpdated Slot that receives signals indicating the UI is updated
 */
void ConfigAttitudeWidget::do_SetDirty()
{
    setDirty(true);
}


void ConfigAttitudeWidget::configureSixPoint()
{
    if (!m_ui->cbCalibrateAccels->isChecked() && !m_ui->cbCalibrateMags->isChecked()) {
        QMessageBox::information(this, "No sensors chosen", "At least one of the sensors must be chosen. \n\nResetting six-point sensor calibration selection.");
        m_ui->cbCalibrateAccels->setChecked(true && board_has_accelerometer);
        m_ui->cbCalibrateMags->setChecked(true && board_has_magnetometer);
    }
    calibration.initialize(m_ui->cbCalibrateAccels->isChecked(), m_ui->cbCalibrateMags->isChecked());
}

void ConfigAttitudeWidget::onCalibrationBusy(bool busy)
{
    // Show the UI is blocking
    if (busy)
        QApplication::setOverrideCursor(Qt::WaitCursor);
    else
        QApplication::restoreOverrideCursor();
}


/**
  @}
  @}
  */
