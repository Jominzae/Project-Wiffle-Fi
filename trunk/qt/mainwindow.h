#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsEllipseItem>
#include <QImage>
#include <QTimer>
#include <QPointF>

#include "rosworker.h"
#include "heatmapper.h"
#include "gradientpalette.h"
#include "autoexplorer.h"
#include "dbmanager.h"
#include "heatlayer.h"
#include "legendbarwidget.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

struct MapMeta
{
    QString imagePath;
    double resolution = 0.05;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double origin_yaw = 0.0;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // ✅ 컨텍스트 기반 모드(Mode 버튼 제거 핵심) - public으로 올려도 되고 private이어도 되지만
    //    cpp/시그니처 꼬임 방지 위해 public이 가장 안전
    enum class Mode { View, Measurement, Query, SimAP };

private slots:
    void onApplyFilter();
    void onRosStatus(const QString &msg);
    void onRobotPose(double x, double y, double yaw);
    void onSample(double x, double y, double yaw, float rssi);
    void flushHeatmap();
    void onLayerHeatmap(bool on);
    void onLayerRobot(bool on);
    void onLayerPins(bool on);
    void onAutoExploreToggled(bool on);
    void onGoalStatusMsg(const QString& msg);
    void onSessionRefresh();
    void onSessionLoad();
    void onSessionDelete();
    void onMeasureStart();
    void onMeasureStop();
    void onSampleToDb(double x, double y, double yaw, float rssi);
    void onSimEnable(bool on);
    void onSimParamsChanged();
    void onClearPins();
    void onMetricChanged(int index);

    void on_btnSessionLoad_clicked();

    // ❌ Mode 버튼 제거 시 더 이상 쓰지 않음(남겨도 되지만 연결은 제거 권장)
    // void onModeChanged();
    // void applyModeUi();

protected:
    void showEvent(QShowEvent* e) override;
    void resizeEvent(QResizeEvent* event) override;
    bool eventFilter(QObject* obj, QEvent* ev) override;

private:
    // ✅ Mode 버튼 제거 후: 모드를 자동으로 계산 + UI/레이어 갱신 단일 진입점
    Mode deriveModeFromContext() const;
    void updateUiByContext();

    void applyLayersPolicy();
    void reloadQueryLayer(const QString& sessionId);
    bool loadStaticMap(const QString& yamlPath);
    bool parseMapYaml(const QString& yamlPath, MapMeta& out);

    bool meterToPixel(double x, double y, int& px, int& py) const;
    bool pixelToMap(int px, int py, double& x_m, double& y_m) const;

    void applyViewTransform();

    bool sceneToMapPixel(const QPointF& scenePos, int& px, int& py) const;
    void onMapClicked(const QPointF& scenePos);

    void initHeatmapLayer();
    float rssiToIntensity01(float rssi) const;
    void onAutoGoal(double x, double y, double yaw);
    void addSimPinAt(double x_m, double y_m);
    void rebuildSimLayer();
    float simRssiAt(double d_m) const;
    float simIntensityAt(double d_m) const;
    float valueToIntensity01(const SampleRow& s) const;
    float simValueToIntensity01(double d_m) const;

private:
    enum class Metric { RSSI, SNR, ApCount, Noise, ThroughputMax };
    Metric currentMetric_ = Metric::RSSI;

    // ✅ Mode는 이제 컨텍스트로 자동 갱신됨
    Mode currentMode_ = Mode::View;

    bool simEnabled_ = false;
    double simTxPower_ = -40.0;
    int simChannel_ = 36;
    int simBandwidth_ = 80;
    HeatLayer simLayer_;

    struct SimPin {
        double x_m = 0.0;
        double y_m = 0.0;
    };
    QVector<SimPin> simPins_;

    QString filterSsid_ = "ALL";
    bool filterThrEnable_ = false;
    int filterThrRssi_ = -60;

    bool measuringDb_ = false;
    QString activeSessionId_;

    DbManager db_;
    HeatLayer queryLayer_;
    QString currentLoadedSession_;

    bool navBusy_ = false;
    QVector<QGraphicsItem*> apPins_;

    AutoExplorer autoExplorer_;

    Ui::MainWindow *ui = nullptr;
    RosWorker *rosThread = nullptr;

    QGraphicsScene *scene = nullptr;
    QGraphicsPixmapItem *mapItem = nullptr;
    QGraphicsEllipseItem *robotItem = nullptr;

    MapMeta mapMeta_;
    QSize mapImageSize_;
    bool mapReady_ = false;
    bool poseReady_ = false;

    QImage heatCanvas_;
    GradientPalette* palette_ = nullptr;
    HeatMapper* heatMapper_ = nullptr;
    QGraphicsPixmapItem* heatItem_ = nullptr;
    QTimer* heatFlushTimer_ = nullptr;
    LegendBarWidget* legendOverlay_ = nullptr;

    void initLegendOverlay();
    void updateLegendOverlayGeometry();
};

#endif
