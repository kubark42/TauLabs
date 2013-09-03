/**
******************************************************************************
*
* @file       textbubbleslider.h
* @author     Tau Labs, http://taulabs.org Copyright (C) 2013.
* @brief      Creates a slider with a text bubble showing the slider value
* @see        The GNU Public License (GPL) Version 3
* @defgroup   Config
* @{
*
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

#include <QApplication>
#include <QDebug>
#include <QPainter>
#include <QStyleOptionSlider>
#include <QDebug>
#include <qmath.h>

#include "textbubbleslider.h"

/**
 * @brief TextBubbleSlider::TextBubbleSlider Constructs a regular text-bubble slider
 * @param parent
 */
TextBubbleSlider::TextBubbleSlider(QWidget *parent) :
    QSlider(parent)
{
    construct();
}

/**
 * @brief TextBubbleSlider::TextBubbleSlider Constructs a text-bubble slider that copys
 * all relevant data from the previous slider
 * @param copySlider
 * @param parent
 */
TextBubbleSlider::TextBubbleSlider(QSlider *copySlider, QWidget *parent) :
    QSlider(parent)
{
    construct();

    // Copy settings
    setSizePolicy(copySlider->sizePolicy());
    setMinimumSize(copySlider->minimumSize());
    setMaximumSize(copySlider->maximumSize());
    setFocusPolicy(copySlider->focusPolicy());
    setOrientation(copySlider->orientation());
    setMaximum(copySlider->maximum());
    setMinimum(copySlider->minimum());
    setToolTip(copySlider->toolTip());
}


/**
 * @brief TextBubbleSlider::construct This function needs to be called from all constructors. It
 * provides a single point where settings can be changed.
 */
void TextBubbleSlider::construct()
{
    font = QFont("Arial", 13);
    fontMetrics = new QFontMetrics(font);

    slideHandleMargin = 2; // This is a dubious way to set the margin. In reality, it should be read from the style sheet.
}

TextBubbleSlider::~TextBubbleSlider()
{
}


/**
 * @brief numIntegerDigits Counts the number of digits in an integer
 * @param number Input integer
 * @return Number of digits in input integer
 */
unsigned int numIntegerDigits(int number)
{
    unsigned int digits = 0;

    // If there is a negative sign, be sure to include it in digit count
    if (number < 0)
        digits = 1;

    while (number) {
        number /= 10;
        digits++;
    }

    return digits;
}


/**
 * @brief TextBubbleSlider::setMaxPixelWidth Sets maximum pixel width for slider handle
 */
void TextBubbleSlider::setMaxPixelWidth()
{
    // Calculate maximum number of digits possible in string
    int maxNumDigits = numIntegerDigits(maximum()) > numIntegerDigits(minimum()) ? numIntegerDigits(maximum()) : numIntegerDigits(minimum());

    // Generate string with maximum pixel width. Suppose that "0" is
    // the widest number in pixels.
    QString maximumWidthString;
    for (int i=0; i<maxNumDigits; i++) {
        maximumWidthString.append("0");
    }

    // Calculate maximum possible pixel width for string.
    maximumFontWidth = fontMetrics->width(QString("%1").arg(maximumWidthString));
    maximumFontHeight = fontMetrics->height();

    // Override stylesheet slider handle width
    slideHandleWidth = maximumFontWidth + 6;
    slideHandleHeight= maximumFontHeight + 1;
    setStyleSheet(QString("QSlider::handle:horizontal { width: %1px; margin: -5px 0;}").arg(slideHandleWidth));
//    setStyleSheet(QString("QSlider::handle:horizontal { width: %1px; margin: -5px 0; border-top: 10px solid transparent;} QSlider::groove {border-image: url(:images/input_selected.png) 0 12 0 10; border-top: 10px;}").arg(slideHandleWidth));
}


/**
 * @brief TextBubbleSlider::setMinimum Reimplements setMinimum. Ensures that the slider
 * handle is the correct size for the text field.
 * @param max maximum
 */
void TextBubbleSlider::setMinimum(int max)
{
    // Pass value on to QSlider
    QSlider::setMinimum(max);

    // Reset handler size
    setMaxPixelWidth();
}


/**
 * @brief TextBubbleSlider::setMaximum Reimplements setMaximum. Ensures that the slider
 * handle is the correct size for the text field.
 * @param max maximum
 */
void TextBubbleSlider::setMaximum(int max)
{
    // Pass value on to QSlider
    QSlider::setMaximum(max);

    // Reset handler size
    setMaxPixelWidth();
}


/**
 * @brief TextBubbleSlider::paintEvent Reimplements QSlider::paintEvent.
 * @param bob
 */
void TextBubbleSlider::paintEvent(QPaintEvent *paintEvent)
{

    if (maximum() != minimum()) {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        int sliderWidth = width();
        int sliderHeight = height();
        int grooveY_origin = ceil(sliderHeight/2.0) - 5;
        int triangleHeight = 5;


        QPolygonF grooveLeft;
        QPolygonF grooveRight;

        double valuePosition;
        double handlePosition;
        if (!invertedAppearance()) {
            handlePosition = (value()-minimum())/(double)(maximum()-minimum()) * (sliderWidth - (slideHandleWidth + slideHandleMargin) - 1);
            valuePosition = ((slideHandleWidth)/2 + slideHandleMargin) + // First part finds handle center...
                    handlePosition; //... and second part moves text with handle
        } else {
            handlePosition = (maximum()-value())/(double)(maximum()-minimum()) * (sliderWidth - (slideHandleWidth + slideHandleMargin) - 1);
            valuePosition = ((slideHandleWidth)/2 + slideHandleMargin) + // First part finds handle center...
                    handlePosition; //... and second part moves text with handle
        }

        // Find the percentage movement
        double handlePercentage = handlePosition / sliderWidth;


        // TODO: Add comment
        float marginLeft;
        float marginRight;
        if (!invertedAppearance()) {
            marginLeft = 1;
            marginRight = 6;
        } else {
            marginLeft = 6;
            marginRight = 1;
        }

        QPointF grooveTopLeft(2, height()/2.0f-marginLeft);
        QPointF grooveBottomLeft(2, height()/2.0f+marginLeft*0+5);
        QPointF grooveTopRight(width()-2, height()/2.0f-marginRight);
        QPointF grooveBottomRight(width()-2, height()/2.0f+marginRight*0+5);

        // Define the groove up to the slider
        grooveLeft << grooveTopLeft <<
                      grooveBottomLeft <<
                      QPointF(valuePosition, grooveBottomLeft.y() + (grooveBottomRight.y()-grooveBottomLeft.y())*handlePercentage) <<
                      QPointF(valuePosition, grooveTopLeft.y() + (grooveTopRight.y()-grooveTopLeft.y())*handlePercentage);

        // Define the groove after the slider
        grooveRight << QPointF(valuePosition, grooveBottomLeft.y() + (grooveBottomRight.y()-grooveBottomLeft.y())*handlePercentage) <<
                       QPointF(valuePosition, grooveTopLeft.y() + (grooveTopRight.y()-grooveTopLeft.y())*handlePercentage) <<
                       grooveTopRight <<
                       grooveBottomRight;
        painter.setPen(QPen(QColor(119,119,119)));

        // Color order depends on inverted appearance
        if (!invertedAppearance()) {
            painter.setBrush(QBrush(QColor(249,117,76)));
            painter.drawPolygon(grooveLeft);
            painter.setBrush(QBrush(Qt::white));
            painter.drawPolygon(grooveRight);
        } else {
            painter.setBrush(QBrush(Qt::white));
            painter.drawPolygon(grooveLeft);
            painter.setBrush(QBrush(QColor(249,117,76)));
            painter.drawPolygon(grooveRight);
        }


        // If mouse cursor is inside handle, change the color
        QPoint p = this->mapFromGlobal(QCursor::pos());
        if (p.x() > ceil(valuePosition-slideHandleWidth/2.0) && p.x() < ceil(valuePosition+slideHandleWidth/2.0) &&
                p.y() > ceil(sliderHeight/2.0 - slideHandleHeight/2.0) && p.y() < ceil(sliderHeight/2.0 + slideHandleHeight/2.0)) {
            painter.setBrush(QBrush(QColor(235,235,235)));
         } else {
            painter.setBrush(QBrush(QColor(196,196,196)));
        }
        painter.drawRoundedRect(QRectF(ceil(valuePosition-slideHandleWidth/2.0), sliderHeight/2.0 - slideHandleHeight/2.0, slideHandleWidth, slideHandleHeight), 3, 3);

        //        QStyle style;
        //        QStyleOptionSlider   opt;
        //        opt.initFrom( this );
        //        opt.minimum        = minimum();
        //        opt.maximum        = maximum();
        //        opt.pageStep       = pageStep();
        //        opt.singleStep     = singleStep();
        //        opt.orientation    = orientation();
        //        opt.sliderPosition = sliderPosition();
        //        opt.sliderValue    = value();
        //        opt.tickInterval   = tickInterval();
        //        opt.tickPosition   = tickPosition();
        //        opt.upsideDown     = invertedAppearance();


    } else {
        // Pass paint event on to QSlider
        QSlider::paintEvent(paintEvent);
/*
        QStyleOptionSlider opt;
        initStyleOption(&opt);

        opt.subControls = QStyle::SC_SliderGroove | QStyle::SC_SliderHandle;

        QRect groove_rect = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderGroove, this);
        QRect rect(groove_rect.left() + 0.2 * groove_rect.width(), groove_rect.top(), 0.6 * groove_rect.width(), groove_rect.height());
        QPainter painter(this);
        painter.fillRect(rect, QBrush(Qt::red));
*/
    }

//    // Pass paint event on to QSlider
//    QSlider::paintEvent(paintEvent);



    /* Add numbers on top of handler */
    // Create painter and set font
    QPainter painter(this);
    painter.setFont(font);

    // Calculate pixel position for text.
    int sliderWidth = width();
    int sliderHeight = height();
    double valuePos;

    if (!invertedAppearance()) {
        valuePos = (slideHandleWidth - maximumFontWidth)/2 + slideHandleMargin + // First part centers text in handle...
                (value()-minimum())/(double)(maximum()-minimum()) * (sliderWidth - (slideHandleWidth + slideHandleMargin) - 1); //... and second part moves text with handle
    } else {
        valuePos = (slideHandleWidth - maximumFontWidth)/2 + slideHandleMargin + // First part centers text in handle...
                (maximum()-value())/(double)(maximum()-minimum()) * (sliderWidth - (slideHandleWidth + slideHandleMargin) - 1); //... and second part moves text with handle
    }

    // Draw neutral value text. Verically center it in the handle
    QString neutralStringWidth = QString("%1").arg(value());
    int textWidth = fontMetrics->width(neutralStringWidth);
    painter.drawText(QRectF(valuePos + maximumFontWidth - textWidth, ceil((sliderHeight - maximumFontHeight)/2.0), textWidth, maximumFontHeight),
                     neutralStringWidth);

return;

    // Draw triangle to represent direction
    int grooveY_origin = ceil(sliderHeight/2.0) - 5;
    int triangleHeight = 5;
    painter.setRenderHint(QPainter::Antialiasing);
    if (!invertedAppearance()) {
        QPolygonF polygon;
        polygon << QPointF(0, grooveY_origin) << QPointF(sliderWidth-1, grooveY_origin) << QPointF(sliderWidth-1, grooveY_origin-triangleHeight) << QPointF(0, grooveY_origin);
        painter.drawPolygon(polygon);
    }
    else {
        QPolygonF polygon;
        polygon << QPointF(0, grooveY_origin) << QPointF(sliderWidth-1, grooveY_origin) << QPointF(0, grooveY_origin-triangleHeight) << QPointF(0, grooveY_origin);
        painter.drawPolygon(polygon);
    }


}
