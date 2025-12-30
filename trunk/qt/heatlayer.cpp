#include "heatlayer.h"
#include <QPixmap>
#include <QColor>

HeatLayer::~HeatLayer() {
    delete mapper_;
    mapper_ = nullptr;
    delete palette_;
    palette_ = nullptr;
}

bool HeatLayer::init(QGraphicsScene* scene, const QSize& size, int z,
                     int radius_px, int opacity)
{
    if (!scene || size.isEmpty()) return false;

    if (radius_px < 1) radius_px = 1;
    if (opacity < 1) opacity = 1;
    if (opacity > 255) opacity = 255;

    canvas_ = QImage(size, QImage::Format_ARGB32);
    canvas_.fill(Qt::transparent);

    delete palette_;
    palette_ = new GradientPalette(256);
    palette_->setColorAt(0.0, QColor(255, 0, 0));
    palette_->setColorAt(0.5, QColor(255, 255, 0));
    palette_->setColorAt(1.0, QColor(0, 255, 0));

    delete mapper_;
    const bool absoluteMode = true;
    const bool cap01 = true;
    mapper_ = new HeatMapper(&canvas_, palette_, radius_px, opacity, absoluteMode, cap01);

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

void HeatLayer::clear() {
    if (!canvas_.isNull()) canvas_.fill(Qt::transparent);
    // data_는 HeatMapper 내부에 남아있을 수 있음.
    // 완전 초기화가 필요하면 HeatMapper를 재생성하는 방식으로 처리.
}

void HeatLayer::addPoint(int px, int py, float intensity01) {
    if (mapper_) mapper_->addPoint(px, py, intensity01);
}

void HeatLayer::flush() {
    if (!mapper_ || !item_) return;
    mapper_->colorize();
    item_->setPixmap(QPixmap::fromImage(canvas_));
}

void HeatLayer::setVisible(bool v) {
    if (item_) item_->setVisible(v);
}
