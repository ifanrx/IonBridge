#ifndef PORT_DATA_H
#define PORT_DATA_H

#include <cstddef>
#include <cstdint>

class PortPowerData {
 protected:
  uint8_t fc_protocol;
  uint8_t temperature;  // unit: degree
  uint16_t current;     // unit: mA
  uint16_t voltage;     // unit: mV
  uint16_t vin_value;   // unit: mV

 public:
  PortPowerData()
      : fc_protocol(0xFF),
        temperature(0xFF),
        current(0xFF),
        voltage(0xFF),
        vin_value(0xFF) {}
  PortPowerData(uint8_t fc_protocol, uint8_t temperature, uint16_t current,
                uint16_t voltage, uint16_t vin_value)
      : fc_protocol(fc_protocol),
        temperature(temperature),
        current(current),
        voltage(voltage),
        vin_value(vin_value) {}

  // Getter methods
  uint8_t GetFCProtocol() const { return fc_protocol; }
  uint8_t GetTemperature() const { return temperature; }
  // Get current value in mA
  uint16_t GetCurrent() const { return current; }
  // Get voltage value in mV
  uint16_t GetVoltage() const { return voltage; }
  uint16_t GetVinValue() const { return vin_value; }
  // Get power value in W
  uint8_t GetPower() const { return voltage * current / 1e6; }

  void SetFCProtocol(uint8_t fc_protocol) { this->fc_protocol = fc_protocol; }
  void SetTemperature(uint8_t temperature) { this->temperature = temperature; }
  void SetCurrent(uint16_t current) { this->current = current; }
  void SetVoltage(uint16_t voltage) { this->voltage = voltage; }
  void SetVinValue(uint16_t vin_value) { this->vin_value = vin_value; }
  void Reset() {
    fc_protocol = 0xFF;
    temperature = 0x0;
    current = 0x0;
    voltage = 0x0;
    vin_value = 0x0;
  }

  void IncrementalAverageTemperature(uint8_t new_val, size_t count) {
    IncrementalAverage(&temperature, new_val, count);
  }
  void IncrementalAverageCurrent(uint16_t new_val, size_t count) {
    IncrementalAverage(&current, new_val, count);
  }
  void IncrementalAverageVoltage(uint16_t new_val, size_t count) {
    IncrementalAverage(&voltage, new_val, count);
  }
  void IncrementalAverageVinValue(uint16_t new_val, size_t count) {
    IncrementalAverage(&vin_value, new_val, count);
  }

 private:
  template <typename T>
  void IncrementalAverage(T *old_avg, T new_val, size_t count) {
    *old_avg = *old_avg + (new_val - *old_avg) / static_cast<float>(count);
  }
};

class PortAverageData : public PortPowerData {
  size_t count;

 public:
  PortAverageData() : PortPowerData() { Reset(); }
  PortAverageData(const PortPowerData &data)
      : PortPowerData(data.GetFCProtocol(), data.GetTemperature(),
                      data.GetCurrent(), data.GetVoltage(), data.GetVinValue()),
        count(0) {}

  void IncrementalAverageData(const PortPowerData &data) {
    fc_protocol = data.GetFCProtocol();
    count++;
    IncrementalAverageTemperature(data.GetTemperature(), count);
    IncrementalAverageCurrent(data.GetCurrent(), count);
    IncrementalAverageVoltage(data.GetVoltage(), count);
    IncrementalAverageVinValue(data.GetVinValue(), count);
  }

  void Reset() {
    PortPowerData::Reset();
    count = 0;
  }
};

class PortStatsData {
  // (value / 8) is actual Volts value -> accurate up to 0.125 V
  uint8_t voltage;
  // (value / 32) is actual Amps value -> accurate up to 0.03125 A
  uint8_t amperage;
  uint8_t temperature;
  uint8_t vin_value;

 public:
  PortStatsData(const PortAverageData &data) {
    voltage = ScaleVoltage(data.GetVoltage());
    amperage = ScaleAmperage(data.GetCurrent());
    temperature = data.GetTemperature();
    vin_value = ScaleVoltage(data.GetVinValue());
  }

  static uint8_t ScaleVoltage(uint16_t voltage) {
    return (uint8_t)(voltage * 8 / 1e3);
  }
  static uint8_t ScaleAmperage(uint16_t amperage) {
    return (uint8_t)(amperage * 32 / 1e3);
  }

  uint8_t GetVoltage() const { return voltage; }
  uint8_t GetAmperage() const { return amperage; }
  uint8_t GetTemperature() const { return temperature; }
  uint8_t GetVinValue() const { return vin_value; }
};

#endif  // PORT_DATA_H
