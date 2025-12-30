#include "heatmapper.h"
#include <QImage>
#include <QColor>
#include <QPainter>
#include <QRadialGradient>
#include <QDebug>
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

    width_ = image->width();
    height_ = image->height();

    data_.resize(width_ * height_);
    data_.fill(0.0);
}

HeatMapper::~HeatMapper()
{
    // 기존 정책 유지(원본은 delete 안함) — 필요하면 delete alphaCanvas_ 하세요.
    // delete alphaCanvas_;
}

qreal HeatMapper::increase(int x, int y, qreal delta)
{
    int index = (y - 1) * width_ + (x - 1);
    qreal v = data_[index] + delta;

    // 절대모드에서 누적을 0~1로 포화하면 색 의미가 유지됨(추천)
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

    // absoluteMode에서는 value를 intensity01(0~1)로 쓴다고 가정
    // caller(MainWindow)에서 이미 0~1로 넣고 있음.
    const qreal count = increase(x, y, value);

    // ✅ 절대모드: max_ 갱신/재그리기 불필요 (항상 동일 기준)
    if (absoluteMode_) {
        drawAlpha(x, y, count, true);
        return;
    }

    // 기존 상대모드 유지
    if (max_ < count) {
        max_ = count;
        redraw();
        return;
    }
    drawAlpha(x, y, count, true);
}

void HeatMapper::redraw()
{
    QColor color(0, 0, 0, 0);
    alphaCanvas_->fill(color);
    mainCanvas_->fill(color);

    int size = data_.size();
    for (int i = 0; i < size; ++i) {
        if (0.0 == data_[i]) continue;
        drawAlpha(i % width_ + 1, i / width_ + 1, data_[i], false);
    }
    colorize();
}

void HeatMapper::drawAlpha(int x, int y, qreal count, bool colorize_now)
{
    qreal ratio = 0.0;

    if (absoluteMode_) {
        // ✅ 절대모드: count 자체가 0~1 스케일이라고 보고 사용
        ratio = count;
        if (ratio < 0.0) ratio = 0.0;
        if (ratio > 1.0) ratio = 1.0;
    } else {
        // 기존 상대모드: max_ 대비 정규화
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
    int alpha = 0;
    int finalAlpha = 0;
    QColor color;

    for (int i = left; i < right; ++i) {
        for (int j = top; j < bottom; ++j) {
            alpha = qAlpha(alphaCanvas_->pixel(i, j));
            if (!alpha)
                continue;

            finalAlpha = (alpha < opacity_ ? alpha : opacity_);
            color = palette_->getColorAt(alpha);

            mainCanvas_->setPixel(i, j, qRgba(color.red(),
                                              color.green(),
                                              color.blue(),
                                              finalAlpha));
        }
    }
}
