#pragma once

#include <QtCharts>

class ActualDemandChart : public QChartView {
 public:
  explicit ActualDemandChart(QWidget *parent);
  void updateActualDemand(double actual, double demand);

  constexpr static const double SAMPLE_COUNT = 250;
  constexpr static const double UPDATE_RATE_SECONDS = 0.02;
  constexpr static const double SAMPLE_COUNT_TIME_FRAME = SAMPLE_COUNT * UPDATE_RATE_SECONDS;

  const int UPDATE_RATE_MILLISECONDS = 50;

  void setRangeY(qreal min, qreal max);

 public slots:
  void replaceBuffers();

 private:
  void init();

  QTimer *updateTimer = nullptr;

  QChart *chartObject = nullptr;

  QValueAxis *axisX = new QValueAxis();
  QValueAxis *axisY = new QValueAxis();

  QLineSeries *pointsActual = nullptr;
  QLineSeries *pointsDemand = nullptr;

  QVector<QPointF> bufferActual;
  QVector<QPointF> bufferDemand;
  QVector<double> valuesX;
};
