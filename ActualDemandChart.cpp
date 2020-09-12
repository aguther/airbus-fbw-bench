#include "ActualDemandChart.h"

ActualDemandChart::ActualDemandChart(
    QWidget *parent
) : QChartView(parent) {
  init();
}

void ActualDemandChart::init() {
  // setup chart
  chartObject = new QChart();
  chartObject->legend()->hide();
  chartObject->setAnimationOptions(QChart::NoAnimation);

  // setup x axis
  axisX->setTickCount(5);
  axisX->setRange(-1 * SAMPLE_COUNT_TIME_FRAME, 0);
  axisX->setLabelsFont(QFont("Cascadia Code", 7));
  chartObject->addAxis(axisX, Qt::AlignBottom);

  // setup y axis
  axisY->setRange(-0.5, 2.5);
  axisY->setTickCount(13);
  axisY->setMinorTickCount(4);
  axisY->setLabelFormat("%+4.2f");
  axisY->setLabelsFont(QFont("Cascadia Code", 7));
  chartObject->addAxis(axisY, Qt::AlignRight);

  // add chart to view
  setChart(chartObject);
  setRenderHint(QPainter::Antialiasing);

  // setup actual points
  pointsActual = new QLineSeries(this);
  QPen pen(Qt::red);
  pen.setWidth(2);
  pointsActual->setPen(pen);
  // add data to chart
  chartObject->addSeries(pointsActual);
  // attach axes
  pointsActual->attachAxis(axisX);
  pointsActual->attachAxis(axisY);

  // setup demand points
  pointsDemand = new QLineSeries(this);
  QPen penLoadDemand(Qt::darkGreen);
  penLoadDemand.setWidth(2);
  pointsDemand->setPen(penLoadDemand);
  // add data to chart
  chartObject->addSeries(pointsDemand);
  // attach axes
  pointsDemand->attachAxis(axisX);
  pointsDemand->attachAxis(axisY);

  valuesX.reserve(SAMPLE_COUNT);
  bufferActual.reserve(SAMPLE_COUNT);
  bufferDemand.reserve(SAMPLE_COUNT);
  for (int i = 0; i < SAMPLE_COUNT; ++i) {
    const double x = (double(i) / SAMPLE_COUNT) * SAMPLE_COUNT_TIME_FRAME - SAMPLE_COUNT_TIME_FRAME;
    valuesX.append(x);
    bufferActual.append(QPointF(x, 0));
    bufferDemand.append(QPointF(x, 0));
  }

  // create and start refresh timer (replaces buffers)
  updateTimer = new QTimer(this);
  QObject::connect(
      updateTimer,
      &QTimer::timeout,
      this,
      &ActualDemandChart::replaceBuffers,
      Qt::ConnectionType::QueuedConnection
  );
  updateTimer->start(UPDATE_RATE_MILLISECONDS);
}

void ActualDemandChart::updateActualDemand(
    double actual,
    double demand
) {
  // add data for actual
  bufferActual.removeFirst();
  bufferActual.append(QPointF(0, actual));

  // add data for demand
  bufferDemand.removeFirst();
  bufferDemand.append(QPointF(0, demand));

  // correct X values
  for (int i = 0; i < SAMPLE_COUNT; ++i) {
    bufferActual[i].setX(valuesX[i]);
    bufferDemand[i].setX(valuesX[i]);
  }
}

void ActualDemandChart::replaceBuffers() {
  // replace series buffers
  pointsActual->replace(bufferActual);
  pointsDemand->replace(bufferDemand);
}

void ActualDemandChart::setRangeY(
    qreal min,
    qreal max
) {
  axisY->setRange(min, max);
}
