#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QTimer>
#include <QVector>
#include "qcustomplot.h"

// Pour le BLE
#include <QBluetoothDeviceDiscoveryAgent>
#include <QBluetoothDeviceInfo>
#include <QLowEnergyController>
#include <QLowEnergyService>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onSerialPortReadyRead();
    void bleNotificationReceived(const QLowEnergyCharacteristic &c, const QByteArray &value);
    void parseSensorData(const QString &data);
    void updateGraphs();
    void on_pushButtonConnect_clicked();
    void on_pushButtonDisconnect_clicked();
    void on_pushButtonRefreshPorts_clicked();
    void on_pushButtonScanBLE_clicked();
    void deviceDiscovered(const QBluetoothDeviceInfo &device);
    void scanFinished();
    void on_radioButtonSerial_clicked();
    void on_radioButtonBLE_clicked();

private:
    void setupGraphs();
    void autoSelectSerialPort();
    void disconnectBLE();

    Ui::MainWindow *ui;
    QSerialPort *serial;
    QTimer *graphTimer;
    QBluetoothDeviceDiscoveryAgent *bleDiscoveryAgent;
    QLowEnergyController *bleController;
    QLowEnergyService *bleService;
    QByteArray serialBuffer;
    double startTime;
    QVector<double> timeData;
    QVector<double> temperatureData;
    QVector<double> humidityData;
    QVector<double> pressureData;


    QList<QBluetoothDeviceInfo> bleDevices;


    void parseSensorDataBLE(const QByteArray &data);
    void parseAccGyroMagData(const QByteArray &data);
};

#endif // MAINWINDOW_H
