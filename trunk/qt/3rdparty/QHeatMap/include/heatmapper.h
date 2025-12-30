#pragma once
#include <QVector>
#include <QString>

class QImage;
class QColor;
class GradientPalette;

class HeatMapper
{
public:
    // absoluteMode=true면 intensity01을 절대 스케일로 사용(권장)
    // cap01=true면 누적값을 1.0에서 포화(clamp)시킴
    HeatMapper(QImage *image, GradientPalette *palette,
               int radius, int opacity,
               bool absoluteMode = false, bool cap01 = false);

    ~HeatMapper();

    void addPoint(int x, int y);
    void addPoint(int x, int y, qreal value);

    void redraw();
    void colorize();
    void colorize(int x, int y);
    void colorize(int left, int top, int right, int bottom);

    void setPalette(GradientPalette *palette);
    qreal getCount(int x, int y);
    void save(const QString &fname);

private:
    qreal increase(int x, int y, qreal delta);
    void drawAlpha(int x, int y, qreal count, bool colorize_now = true);

private:
    int radius_ = 18;
    int opacity_ = 160;

    // 기존: 상대 정규화용 최대값
    qreal max_ = 1.0;

    // 추가: 절대모드
    bool absoluteMode_ = false;
    bool cap01_ = false;

    int width_ = 0;
    int height_ = 0;

    QVector<qreal> data_;

    GradientPalette *palette_ = nullptr;
    QImage *mainCanvas_ = nullptr;
    QImage *alphaCanvas_ = nullptr;
};
