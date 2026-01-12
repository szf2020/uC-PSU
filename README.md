# Microcontrolled) CC/CV Power Supply Unit

Via [ngscopeclient](https://www.ngscopeclient.org/manual/PowerSupplyDrivers.html) `rigol_dp8xx` driver's SCPI-based protocol

![](https://github.com/user-attachments/assets/768120eb-cf5a-4ea2-833c-9a4e1426ecf7)

## It appears
 one doesn't need no digipot nor DAC. PWM + RC-filter works pretty well

**Voltage:** is pretty linear until 1.25 which is XL4015 internal refence voltage value
![duty-mv](https://github.com/user-attachments/assets/fc2cdb0e-eb81-4f2e-8c93-992bec833e58)

**Current:** has non-linearity issues in lower adc values (and I yet have no clue why)
![duty-ma](https://github.com/user-attachments/assets/084e413d-2e9d-436b-a97e-53555233db56)
