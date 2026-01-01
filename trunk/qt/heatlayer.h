#pragma once
#include <QImage>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QVector>

class HeatMapper;
class GradientPalette;

class HeatLayer
{
public:
    ~HeatLayer();

    bool init(QGraphicsScene* scene, const QSize& size, int z, int radius_px, int opacity);

    void clear(bool clearCanvasToo = true);
    void setVisible(bool v);

    //  평균용
    void clearStats(); // sum/cnt/active 초기화
    void addPointMean(int px, int py, float intensity01); // 1-based 입력(현재 프로젝트 그대로)
    void flushMean(); // 평균 기반 렌더

    // (옵션) 기존 방식 유지하고 싶으면 그대로 둠
    void addPoint(int px, int py, float intensity01);
    void flush();

    bool isReady() const { return mapper_ && item_; }

private:
    void clearCanvas();

private:
    QImage canvas_;
    QGraphicsPixmapItem* item_ = nullptr;

    HeatMapper* mapper_ = nullptr;
    GradientPalette* palette_ = nullptr;

    int radius_px_ = 10;
    int opacity_ = 160;

    //  평균 집계 버퍼
    int w_ = 0;
    int h_ = 0;
    QVector<float> sum_;         // size = w*h
    QVector<uint32_t> cnt_;      // size = w*h
    QVector<int> active_idx_;    // 방문한 픽셀 인덱스 리스트
    QVector<uint8_t> active_flag_; // 중복 방지(0/1)
};
