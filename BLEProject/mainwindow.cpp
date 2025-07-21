#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDateTime>
#include <QStringList>
#include <QSerialPortInfo>
#include <QRegularExpression>
#include <QRegularExpressionMatch>
#include <QDataStream>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
    ui(new Ui::MainWindow),
    serial(new QSerialPort(this)),
    graphTimer(new QTimer(this)),
    bleDiscoveryAgent(new QBluetoothDeviceDiscoveryAgent(this)),
    bleController(nullptr),
    bleService(nullptr),
    startTime(QDateTime::currentDateTime().toMSecsSinceEpoch() / 1000.0)
{
    ui->setupUi(this);
    setupGraphs();
    connect(serial, &QSerialPort::readyRead, this, &MainWindow::onSerialPortReadyRead);
    graphTimer->setInterval(1000);
    connect(graphTimer, &QTimer::timeout, this, &MainWindow::updateGraphs);
    graphTimer->start();
    connect(bleDiscoveryAgent, &QBluetoothDeviceDiscoveryAgent::deviceDiscovered, this, &MainWindow::deviceDiscovered);
    connect(bleDiscoveryAgent, &QBluetoothDeviceDiscoveryAgent::finished, this, &MainWindow::scanFinished);
    on_pushButtonRefreshPorts_clicked();
    autoSelectSerialPort();
    ui->radioButtonSerial->setChecked(true);
    ui->comboBoxSerialPorts->setEnabled(true);
    ui->pushButtonRefreshPorts->setEnabled(true);
    ui->pushButtonConnect->setEnabled(true);
    ui->pushButtonDisconnect->setEnabled(true);
    ui->pushButtonScanBLE->setEnabled(false);
    ui->listWidgetBLEDevices->setEnabled(false);
    ui->labelStatus->setText("Statut : Non connecté");
}

MainWindow::~MainWindow()
{
    if (serial->isOpen())
        serial->close();
    disconnectBLE();
    delete ui;
}

void MainWindow::setupGraphs()
{
    ui->widgetTempPlot->addGraph();
    ui->widgetTempPlot->xAxis->setLabel("Temps (s)");
    ui->widgetTempPlot->yAxis->setLabel("Température (°C)");
    ui->widgetTempPlot->xAxis->setRange(0, 60);
    ui->widgetTempPlot->yAxis->setRange(-10, 50);
    ui->widgetHumidityPlot->addGraph();
    ui->widgetHumidityPlot->xAxis->setLabel("Temps (s)");
    ui->widgetHumidityPlot->yAxis->setLabel("Humidité (%)");
    ui->widgetHumidityPlot->xAxis->setRange(0, 60);
    ui->widgetHumidityPlot->yAxis->setRange(0, 100);
    ui->widgetPressurePlot->addGraph();
    ui->widgetPressurePlot->xAxis->setLabel("Temps (s)");
    ui->widgetPressurePlot->yAxis->setLabel("Pression (hPa)");
    ui->widgetPressurePlot->xAxis->setRange(0, 60);
    ui->widgetPressurePlot->yAxis->setRange(900, 1100);
}

void MainWindow::autoSelectSerialPort()
{
    const auto ports = QSerialPortInfo::availablePorts();
    if (!ports.isEmpty()) {
        QString portName = ports.first().portName();
        int index = ui->comboBoxSerialPorts->findText(portName);
        if (index != -1)
            ui->comboBoxSerialPorts->setCurrentIndex(index);
    }
}

void MainWindow::onSerialPortReadyRead()
{
    QByteArray data = serial->readAll();
    serialBuffer.append(data);
    int index;
    while ((index = serialBuffer.indexOf("\r\n")) != -1) {
        QString line = serialBuffer.left(index).trimmed();
        serialBuffer.remove(0, index + 2);
        parseSensorData(line);
    }
}

void MainWindow::bleNotificationReceived(const QLowEnergyCharacteristic &c, const QByteArray &value)
{
    QString uuidStr = c.uuid().toString().toLower();
    if (uuidStr == "{001c0000-0001-11e1-ac36-0002a5d5c51b}") {
        if (value.size() == 10)
            parseSensorDataBLE(value);
    }
    else if (uuidStr == "{00e00000-0001-11e1-ac36-0002a5d5c51b}") {
        if (value.size() == 20)
            parseAccGyroMagData(value);
    }
    else {
        QString data = QString::fromUtf8(value);
        parseSensorData(data);
    }
}

void MainWindow::parseSensorData(const QString &data)
{
    QRegularExpression rePressure("P:\\s*([0-9]+\\.?[0-9]*)");
    QRegularExpression reTemperature("T:\\s*([0-9]+\\.?[0-9]*)");
    QRegularExpression reHumidity("H:\\s*([0-9]+\\.?[0-9]*)");
    QRegularExpressionMatch matchPressure = rePressure.match(data);
    QRegularExpressionMatch matchTemperature = reTemperature.match(data);
    QRegularExpressionMatch matchHumidity = reHumidity.match(data);
    if (!matchPressure.hasMatch() || !matchTemperature.hasMatch() || !matchHumidity.hasMatch())
        return;
    QString pressureStr = matchPressure.captured(1).trimmed();
    QString temperatureStr = matchTemperature.captured(1).trimmed();
    QString humidityStr = matchHumidity.captured(1).trimmed();
    bool ok = false;
    double pressure = pressureStr.toDouble(&ok);
    if (!ok) return;
    double temperature = temperatureStr.toDouble(&ok);
    if (!ok) return;
    double humidity = humidityStr.toDouble(&ok);
    if (!ok) return;
    ui->labelPressureValue->setText(QString::number(pressure, 'f', 2));
    ui->labelTempValue->setText(QString::number(temperature, 'f', 1));
    ui->labelHumidityValue->setText(QString::number(humidity, 'f', 1));
    double now = QDateTime::currentDateTime().toMSecsSinceEpoch() / 1000.0 - startTime;
    timeData.append(now);
    temperatureData.append(temperature);
    humidityData.append(humidity);
    pressureData.append(pressure);
    while (!timeData.isEmpty() && now - timeData.first() > 60) {
        timeData.removeFirst();
        temperatureData.removeFirst();
        humidityData.removeFirst();
        pressureData.removeFirst();
    }
}

void MainWindow::parseSensorDataBLE(const QByteArray &data)
{
    if (data.size() != 10)
        return;
    QDataStream stream(data);
    stream.setByteOrder(QDataStream::LittleEndian);
    quint16 dummy;
    qint32 pressInt;
    qint16 humInt;
    qint16 tempInt;
    stream >> dummy >> pressInt >> humInt >> tempInt;
    double pressure = pressInt / 100.0;
    double temperature = tempInt / 10.0;
    double humidity = humInt / 10.0;
    ui->labelPressureValue->setText(QString::number(pressure, 'f', 2));
    ui->labelTempValue->setText(QString::number(temperature, 'f', 1));
    ui->labelHumidityValue->setText(QString::number(humidity, 'f', 1));
    double now = QDateTime::currentDateTime().toMSecsSinceEpoch() / 1000.0 - startTime;
    timeData.append(now);
    temperatureData.append(temperature);
    humidityData.append(humidity);
    pressureData.append(pressure);
    while (!timeData.isEmpty() && now - timeData.first() > 60) {
        timeData.removeFirst();
        temperatureData.removeFirst();
        humidityData.removeFirst();
        pressureData.removeFirst();
    }
}

void MainWindow::parseAccGyroMagData(const QByteArray &data)
{
    if (data.size() != 20)
        return;
    QDataStream stream(data);
    stream.setByteOrder(QDataStream::LittleEndian);
    quint16 timeStamp;
    qint16 ax, ay, az;
    qint16 gx, gy, gz;
    qint16 mx, my, mz;
    stream >> timeStamp >> ax >> ay >> az >> gx >> gy >> gz >> mx >> my >> mz;
}

void MainWindow::updateGraphs()
{
    ui->widgetTempPlot->graph(0)->setData(timeData, temperatureData);
    if (!timeData.isEmpty())
        ui->widgetTempPlot->xAxis->setRange(timeData.first(), timeData.last());
    ui->widgetTempPlot->replot();
    ui->widgetHumidityPlot->graph(0)->setData(timeData, humidityData);
    if (!timeData.isEmpty())
        ui->widgetHumidityPlot->xAxis->setRange(timeData.first(), timeData.last());
    ui->widgetHumidityPlot->replot();
    ui->widgetPressurePlot->graph(0)->setData(timeData, pressureData);
    if (!timeData.isEmpty())
        ui->widgetPressurePlot->xAxis->setRange(timeData.first(), timeData.last());
    ui->widgetPressurePlot->replot();
}

void MainWindow::on_pushButtonConnect_clicked()
{
    if (ui->radioButtonSerial->isChecked()) {
        if (serial->isOpen())
            serial->close();
        QString portName = ui->comboBoxSerialPorts->currentText();
        if (portName.isEmpty()) {
            ui->labelStatus->setText("Statut : Aucun port série détecté");
            return;
        }
        serial->setPortName(portName);
        serial->setBaudRate(QSerialPort::Baud115200);
        serial->setDataBits(QSerialPort::Data8);
        serial->setParity(QSerialPort::NoParity);
        serial->setStopBits(QSerialPort::OneStop);
        serial->setFlowControl(QSerialPort::NoFlowControl);
        if (serial->open(QIODevice::ReadOnly))
            ui->labelStatus->setText("Statut : Connecté (Port Série)");
        else
            ui->labelStatus->setText("Statut : Erreur de connexion série");
    } else if (ui->radioButtonBLE->isChecked()) {
        if (ui->listWidgetBLEDevices->currentRow() < 0) {
            ui->labelStatus->setText("Statut : Sélectionnez un appareil BLE");
            return;
        }
        QBluetoothDeviceInfo device = bleDevices.at(ui->listWidgetBLEDevices->currentRow());
        bleController = QLowEnergyController::createCentral(device, this);
        connect(bleController, &QLowEnergyController::connected, this, [this]() {
            ui->labelStatus->setText("Statut : Connecté (BLE)");
            bleController->discoverServices();
        });
        connect(bleController, &QLowEnergyController::disconnected, this, [this]() {
            ui->labelStatus->setText("Statut : Déconnecté (BLE)");
        });
        connect(bleController, &QLowEnergyController::serviceDiscovered, this, [this](const QBluetoothUuid &serviceUuid) {});
        connect(bleController, &QLowEnergyController::discoveryFinished, this, [this]() {
            if (!bleController->services().isEmpty()) {
                bleService = bleController->createServiceObject(bleController->services().first(), this);
                if (bleService) {
                    connect(bleService, &QLowEnergyService::stateChanged, this, [this](QLowEnergyService::ServiceState newState) {
                        if(newState == QLowEnergyService::RemoteServiceDiscovered) {
                            const QList<QLowEnergyCharacteristic> characteristics = bleService->characteristics();
                            for (const QLowEnergyCharacteristic &ch : characteristics) {
                                if (ch.properties() & QLowEnergyCharacteristic::Notify) {
                                    QLowEnergyDescriptor configDescriptor = ch.descriptor(
                                        QBluetoothUuid("{00002902-0000-1000-8000-00805f9b34fb}"));
                                    if (configDescriptor.isValid())
                                        bleService->writeDescriptor(configDescriptor, QByteArray::fromHex("0100"));
                                }
                            }
                        }
                    });
                    connect(bleService, &QLowEnergyService::characteristicChanged, this, &MainWindow::bleNotificationReceived);
                    bleService->discoverDetails();
                }
            }
        });
        bleController->connectToDevice();
        ui->labelStatus->setText("Statut : Tentative de connexion BLE ...");
    }
}

void MainWindow::on_pushButtonDisconnect_clicked()
{
    if (ui->radioButtonSerial->isChecked()) {
        if (serial->isOpen())
            serial->close();
        ui->labelStatus->setText("Statut : Déconnecté (Port Série)");
    } else if (ui->radioButtonBLE->isChecked()) {
        disconnectBLE();
        ui->labelStatus->setText("Statut : Déconnecté (BLE)");
    }
}

void MainWindow::on_pushButtonRefreshPorts_clicked()
{
    ui->comboBoxSerialPorts->clear();
    const auto ports = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &portInfo : ports)
        ui->comboBoxSerialPorts->addItem(portInfo.portName());
    autoSelectSerialPort();
}

void MainWindow::on_pushButtonScanBLE_clicked()
{
    ui->listWidgetBLEDevices->clear();
    bleDevices.clear();
    bleDiscoveryAgent->start(QBluetoothDeviceDiscoveryAgent::LowEnergyMethod);
    ui->labelStatus->setText("Statut : Scan BLE en cours...");
}

void MainWindow::deviceDiscovered(const QBluetoothDeviceInfo &device)
{
    if (device.coreConfigurations() & QBluetoothDeviceInfo::LowEnergyCoreConfiguration) {
        bleDevices.append(device);
        ui->listWidgetBLEDevices->addItem(device.name().isEmpty() ? device.address().toString() : device.name());
    }
}

void MainWindow::scanFinished()
{
    ui->labelStatus->setText("Statut : Scan BLE terminé");
}

void MainWindow::on_radioButtonSerial_clicked()
{
    if (bleController)
        disconnectBLE();
    ui->comboBoxSerialPorts->setEnabled(true);
    ui->pushButtonRefreshPorts->setEnabled(true);
    ui->pushButtonConnect->setEnabled(true);
    ui->pushButtonDisconnect->setEnabled(true);
    ui->pushButtonScanBLE->setEnabled(false);
    ui->listWidgetBLEDevices->setEnabled(false);
    ui->labelStatus->setText("Statut : Mode Port Série");
}

void MainWindow::on_radioButtonBLE_clicked()
{
    if (serial->isOpen())
        serial->close();
    ui->pushButtonScanBLE->setEnabled(true);
    ui->listWidgetBLEDevices->setEnabled(true);
    ui->comboBoxSerialPorts->setEnabled(false);
    ui->pushButtonRefreshPorts->setEnabled(false);
    ui->pushButtonConnect->setEnabled(true);
    ui->pushButtonDisconnect->setEnabled(true);
    on_pushButtonScanBLE_clicked();
    ui->labelStatus->setText("Statut : Mode Bluetooth LE");
}

void MainWindow::disconnectBLE()
{
    if (bleController) {
        bleController->disconnectFromDevice();
        bleController->deleteLater();
        bleController = nullptr;
    }
    if (bleService) {
        bleService->deleteLater();
        bleService = nullptr;
    }
}
