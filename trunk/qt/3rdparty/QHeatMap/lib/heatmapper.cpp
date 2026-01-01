#include "heatmapper.h"
#include <QColor>
#include <QPainter>
#include <QRadialGradient>
#include <QtGlobal>
#include "gradientpalette.h"

HeatMapper::HeatMapper(QImage *image, GradientPalette *palette,
                       int radius, int opacity,
                       bool absoluteMode, bool cap01)
    : radius_(radius),
    opacity_(opacity),
    max_(1.0),
    absoluteMode_(absoluteMode),
    cap01_(cap01)
{
    Q_ASSERT(image);
    Q_ASSERT(palette);

    palette_ = palette;
    mainCanvas_ = image;

    alphaCanvas_ = new QImage(image->size(), QImage::Format_ARGB32);
    Q_ASSERT(alphaCanvas_);
    alphaCanvas_->fill(QColor(0, 0, 0, 0));

    width_  = image->width();
    height_ = image->height();

    data_.resize(width_ * height_);
    data_.fill(0.0);
}

HeatMapper::~HeatMapper()
{
    delete alphaCanvas_;
    alphaCanvas_ = nullptr;
}

void HeatMapper::clearAlpha()
{
    if (!alphaCanvas_ || !mainCanvas_) return;
    const QColor clear(0,0,0,0);
    alphaCanvas_->fill(clear);
    mainCanvas_->fill(clear);
}

qreal HeatMapper::increase(int x, int y, qreal delta)
{
    int index = (y - 1) * width_ + (x - 1);
    qreal v = data_[index] + delta;

    if (cap01_) {
        if (v < 0.0) v = 0.0;
        if (v > 1.0) v = 1.0;
    }
    data_[index] = v;
    return data_[index];
}

void HeatMapper::addPoint(int x, int y)
{
    addPoint(x, y, 1.0);
}

void HeatMapper::addPoint(int x, int y, qreal value)
{
    if (x <= 0 || y <= 0 || x > width_ || y > height_) return;
    if (value <= 0.0) return;

    const qreal count = increase(x, y, value);

    if (absoluteMode_) {
        drawAlpha(x, y, count, true);
        return;
    }

    if (max_ < count) {
        max_ = count;
        redraw();
        return;
    }
    drawAlpha(x, y, count, true);
}

void HeatMapper::redraw()
{
    QColor clear(0, 0, 0, 0);
    alphaCanvas_->fill(clear);
    mainCanvas_->fill(clear);

    int size = data_.size();
    for (int i = 0; i < size; ++i) {
        if (0.0 == data_[i]) continue;
        drawAlpha(i % width_ + 1, i / width_ + 1, data_[i], false);
    }
    colorize();
}

void HeatMapper::stampAbs(int x, int y, qreal ratio01, bool colorize_now)
{
    //  ratio01(0~1) 그대로 라디얼 강도로 사용
    if (!alphaCanvas_ || !mainCanvas_) return;
    if (x <= 0 || y <= 0 || x > width_ || y > height_) return;

    if (ratio01 < 0.0) ratio01 = 0.0;
    if (ratio01 > 1.0) ratio01 = 1.0;

    const int alpha = int(ratio01 * 255.0);

    QRadialGradient gradient(x, y, radius_);
    gradient.setColorAt(0, QColor(0, 0, 0, alpha));
    gradient.setColorAt(1, QColor(0, 0, 0, 0));

    QPainter painter(alphaCanvas_);
    painter.setPen(Qt::NoPen);
    painter.setBrush(gradient);
    painter.drawEllipse(QPoint(x, y), radius_, radius_);

    if (colorize_now) colorize(x, y);
}

void HeatMapper::drawAlpha(int x, int y, qreal count, bool colorize_now)
{
    qreal ratio = 0.0;

    if (absoluteMode_) {
        ratio = count;
        if (ratio < 0.0) ratio = 0.0;
        if (ratio > 1.0) ratio = 1.0;
    } else {
        ratio = (max_ > 0.0) ? (count / max_) : 0.0;
        if (ratio < 0.0) ratio = 0.0;
        if (ratio > 1.0) ratio = 1.0;
    }

    int alpha = int(ratio * 255.0);

    QRadialGradient gradient(x, y, radius_);
    gradient.setColorAt(0, QColor(0, 0, 0, alpha));
    gradient.setColorAt(1, QColor(0, 0, 0, 0));

    QPainter painter(alphaCanvas_);
    painter.setPen(Qt::NoPen);
    painter.setBrush(gradient);
    painter.drawEllipse(QPoint(x, y), radius_, radius_);

    if (colorize_now)
        colorize(x, y);
}

void HeatMapper::colorize()
{
    colorize(0, 0, width_, height_);
}

void HeatMapper::colorize(int x, int y)
{
    int left = x - radius_;
    int top = y - radius_;
    int right = x + radius_;
    int bottom = y + radius_;

    if (left < 0) left = 0;
    if (top < 0) top = 0;
    if (right > width_) right = width_;
    if (bottom > height_) bottom = height_;

    colorize(left, top, right, bottom);
}

void HeatMapper::colorize(int left, int top, int right, int bottom)
{
    if (!alphaCanvas_ || !mainCanvas_ || !palette_) return;

    for (int i = left; i < right; ++i) {
        for (int j = top; j < bottom; ++j) {
            const int a = qAlpha(alphaCanvas_->pixel(i, j));
            if (!a) continue;

            const int finalAlpha = (a < opacity_ ? a : opacity_);
            const QColor c = palette_->getColorAt(a); // a: 0~255

            mainCanvas_->setPixel(i, j, qRgba(c.red(), c.green(), c.blue(), finalAlpha));
        }
    }
}
