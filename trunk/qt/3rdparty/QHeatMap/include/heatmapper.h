#pragma once
#include <QImage>
#include <QVector>

class GradientPalette;

class HeatMapper
{
public:
    HeatMapper(QImage *image, GradientPalette *palette,
               int radius, int opacity,
               bool absoluteMode, bool cap01);
    ~HeatMapper();

    void addPoint(int x, int y);
    void addPoint(int x, int y, qreal value);

    void redraw();

    //  NEW: 평균/재렌더링용
    void clearAlpha(); // alphaCanvas_ + mainCanvas_ 투명 초기화
    void stampAbs(int x, int y, qreal ratio01, bool colorize_now=false); // ratio01(0~1)로 라디얼 찍기

    void colorize();
    void colorize(int x, int y);
    void colorize(int left, int top, int right, int bottom);

private:
    qreal increase(int x, int y, qreal delta);
    void drawAlpha(int x, int y, qreal count, bool colorize_now);

private:
    int radius_;
    int opacity_;
    qreal max_;
    bool absoluteMode_;
    bool cap01_;

    GradientPalette *palette_ = nullptr;
    QImage *mainCanvas_ = nullptr;
    QImage *alphaCanvas_ = nullptr;

    int width_ = 0;
    int height_ = 0;
    QVector<qreal> data_;
};
