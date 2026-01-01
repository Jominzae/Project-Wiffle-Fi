#include "heatlayer.h"
#include "heatmapper.h"
#include "gradientpalette.h"

#include <QPixmap>
#include <QColor>
#include <QtGlobal>

HeatLayer::~HeatLayer() {
    delete mapper_;
    delete palette_;
}

bool HeatLayer::init(QGraphicsScene* scene, const QSize& size, int z, int radius_px, int opacity)
{
    if (!scene || size.isEmpty()) return false;

    radius_px_ = qMax(1, radius_px);
    opacity_   = qBound(1, opacity, 255);

    canvas_ = QImage(size, QImage::Format_ARGB32);
    canvas_.fill(Qt::transparent);

    w_ = size.width();
    h_ = size.height();

    //  평균 버퍼 초기화
    sum_.assign(w_ * h_, 0.0f);
    cnt_.assign(w_ * h_, 0u);
    active_idx_.clear();
    active_flag_.assign(w_ * h_, 0);

    delete palette_;
    palette_ = new GradientPalette(256);
    palette_->setColorAt(0.0, QColor(255, 0, 0));
    palette_->setColorAt(0.5, QColor(255, 255, 0));
    palette_->setColorAt(1.0, QColor(0, 255, 0));

    delete mapper_;
    mapper_ = new HeatMapper(&canvas_, palette_, radius_px_, opacity_, /*absoluteMode=*/true, /*cap01=*/true);

    if (!item_) {
        item_ = scene->addPixmap(QPixmap::fromImage(canvas_));
        item_->setZValue(z);
        item_->setPos(0, 0);
    } else {
        item_->setPixmap(QPixmap::fromImage(canvas_));
        item_->setZValue(z);
    }
    return true;
}

void HeatLayer::clearCanvas()
{
    if (canvas_.isNull()) return;
    canvas_.fill(Qt::transparent);
    if (item_) item_->setPixmap(QPixmap::fromImage(canvas_));
}

void HeatLayer::clearStats()
{
    if (w_ <= 0 || h_ <= 0) return;
    sum_.fill(0.0f);
    cnt_.fill(0u);
    active_idx_.clear();
    active_flag_.fill(0);
}

void HeatLayer::clear(bool clearCanvasToo)
{
    //  평균 통계도 같이 초기화
    clearStats();
    if (mapper_) mapper_->clearAlpha(); // alpha/main clear
    if (clearCanvasToo) clearCanvas();
}

void HeatLayer::setVisible(bool v)
{
    if (item_) item_->setVisible(v);
}

// ===== 평균용 =====
void HeatLayer::addPointMean(int px, int py, float intensity01)
{
    if (w_ <= 0 || h_ <= 0) return;
    if (px <= 0 || py <= 0 || px > w_ || py > h_) return;

    if (intensity01 < 0.f) intensity01 = 0.f;
    if (intensity01 > 1.f) intensity01 = 1.f;

    const int x0 = px - 1;
    const int y0 = py - 1;
    const int idx = y0 * w_ + x0;

    sum_[idx] += intensity01;
    cnt_[idx] += 1;

    if (!active_flag_[idx]) {
        active_flag_[idx] = 1;
        active_idx_.push_back(idx);
    }
}

void HeatLayer::flushMean()
{
    if (!mapper_ || !item_ || canvas_.isNull()) return;

    //  매 프레임: alpha/main clear -> active 픽셀 stamp -> colorize 1회
    mapper_->clearAlpha();

    for (int idx : active_idx_) {
        const uint32_t c = cnt_[idx];
        if (c == 0) continue;

        const float avg = sum_[idx] / (float)c;

        const int x0 = idx % w_;
        const int y0 = idx / w_;
        const int px = x0 + 1; // HeatMapper는 1-based 좌표를 쓰는 기존 정책 유지
        const int py = y0 + 1;

        mapper_->stampAbs(px, py, avg, /*colorize_now=*/false);
    }

    mapper_->colorize();
    item_->setPixmap(QPixmap::fromImage(canvas_));
}

// ===== 기존 방식(원하면 유지) =====
void HeatLayer::addPoint(int px, int py, float intensity01)
{
    if (mapper_) mapper_->addPoint(px, py, intensity01);
}

void HeatLayer::flush()
{
    if (!mapper_ || !item_) return;
    mapper_->colorize();
    item_->setPixmap(QPixmap::fromImage(canvas_));
}
