
#include <BitBang_I2C.h>

#define SDA_PIN 3
#define SCL_PIN 4

BBI2C bbi2c;
#define BME680_ADDR 0x77
// Calibration vars
uint16_t _T1, _P1, _H1, _H2;
int16_t _T2, _P2, _P4, _P5, _P8, _P9, _G2;
int8_t _T3, _P3, _P6, _P7, _H3, _H4, _H5, _H7, _G1, _G3;
uint8_t _P10, _H6;

void GetCalibration(void)
{
  /*************************************
  ** Temperature related coefficients **
  *************************************/
  uint8_t coeff_arr1[25] = {0};
  uint8_t coeff_arr2[16] = {0};

  I2CReadRegister(&bbi2c, BME680_ADDR, 0x89, coeff_arr1, sizeof(coeff_arr1));
  I2CReadRegister(&bbi2c, BME680_ADDR, 0xe1, coeff_arr2, sizeof(coeff_arr2));

// Temp adjustments (little endian order)
  _T1 = (uint16_t)(coeff_arr2[8] + (coeff_arr2[9] << 8));
  _T2 = (int16_t)(coeff_arr1[1] + (coeff_arr1[2] << 8));
  _T3 = (int8_t)(coeff_arr1[3]);
// Humidity adjustments
  _H1 = (uint16_t)(((uint16_t)coeff_arr2[2] << 4) |
                   (coeff_arr2[1] & 0xf));
  _H2 = (uint16_t)(((uint16_t)coeff_arr2[0] << 4) |
                   ((coeff_arr2[1]) >> 4));
  _H3 = (int8_t)coeff_arr2[3];
  _H4 = (int8_t)coeff_arr2[4];
  _H5 = (int8_t)coeff_arr2[5];
  _H6 = (uint8_t)coeff_arr2[6];
  _H7 = (int8_t)coeff_arr2[7];

// Pressure adjustments
  _P1  = (uint16_t)((coeff_arr1[6] << 8) + coeff_arr1[5]);
  _P2  = (int16_t)((coeff_arr1[8] << 8) + coeff_arr1[7]);
  _P3  = (int8_t)coeff_arr1[9];
  _P4  = (int16_t)((coeff_arr1[12] << 8) + coeff_arr1[11]);
  _P5  = (int16_t)((coeff_arr1[14] << 8) + coeff_arr1[13]);
  _P6  = (int8_t)(coeff_arr1[16]);
  _P7  = (int8_t)(coeff_arr1[15]);
  _P8  = (int16_t)((coeff_arr1[20] << 8) + coeff_arr1[19]);
  _P9  = (int16_t)((coeff_arr1[22] << 8) + coeff_arr1[21]);
  _P10 = (uint8_t)(coeff_arr1[23]);

// Gas adjustments
  _G1 = (int8_t)coeff_arr2[12];
  _G2 = (int16_t)((coeff_arr2[11] << 8) + coeff_arr2[10]);
  _G3 = (int8_t)coeff_arr2[13];
//  uint8_t temp_var = 0;
//  getData(BME680_ADDR_RES_HEAT_RANGE_ADDR, temp_var);
//  _res_heat_range = ((temp_var & BME680_RHRANGE_MSK) / 16);
//  getData(BME680_ADDR_RES_HEAT_VAL_ADDR, temp_var);
//  _res_heat = (int8_t)temp_var;
//  getData(BME680_ADDR_RANGE_SW_ERR_ADDR, temp_var);
//  _rng_sw_err = ((int8_t)temp_var & (int8_t)BME680_RSERROR_MSK) / 16;
} /* GetCalibration() */

void BME680GetReadings(void)
{
  uint8_t ucTemp[15];
  uint32_t adc_temp, adc_hum, adc_pres;
  int32_t _tfine;
  int32_t _Humidity, _Pressure;
  int16_t _Temperature;
  int64_t var1, var2, var3, var4, var5, var6, temp_scaled;

  I2CReadRegister(&bbi2c, BME680_ADDR, 0x74, &ucTemp[1], 1); // read CTRL_MEASURE_REG (temp/pressure)
  ucTemp[1] |= 1; // start first reading
  ucTemp[0] = 0x74; // write it back to the same register
  I2CWrite(&bbi2c, BME680_ADDR, ucTemp, 2);
  delay(100);
  I2CReadRegister(&bbi2c, BME680_ADDR, 0x1d, ucTemp, 15); // read all raw sensor values at once
//  Serial.print("Status reg 0x1d = 0x");
//  Serial.println(ucTemp[0], HEX);

  // Calculate the temperature
  adc_temp = (uint32_t)(((uint32_t)ucTemp[5] * 4096) | ((uint32_t)ucTemp[6] * 16) |
                        ((uint32_t)ucTemp[7] / 16));  // put the 3 bytes of Temperature
  var1         = ((int32_t)adc_temp >> 3) - ((int32_t)_T1 << 1);  // Perform calibration/adjustment
  var2         = (var1 * (int32_t)_T2) >> 11;                     // of Temperature values according
  var3         = ((var1 >> 1) * (var1 >> 1)) >> 12;               // to formula defined by Bosch
  var3         = ((var3) * ((int32_t)_T3 << 4)) >> 14;
  _tfine       = (int32_t)(var2 + var3);
  _Temperature = (int16_t)(((_tfine * 5) + 128) >> 8);

  // Calculate the pressure
  adc_pres = (uint32_t)(((uint32_t)ucTemp[2] * 4096) | ((uint32_t)ucTemp[3] * 16) |
                        ((uint32_t)ucTemp[4] / 16));  // put the 3 bytes of Pressure
  var1 = (((int32_t)_tfine) >> 1) - 64000;
  var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)_P6) >> 2;
  var2 = var2 + ((var1 * (int32_t)_P5) << 1);
  var2 = (var2 >> 2) + ((int32_t)_P4 << 16);
  var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * ((int32_t)_P3 << 5)) >> 3) +
         (((int32_t)_P2 * var1) >> 1);
  var1      = var1 >> 18;
  var1      = ((32768 + var1) * (int32_t)_P1) >> 15;
  _Pressure = 1048576 - adc_pres;
  _Pressure = (int32_t)((_Pressure - (var2 >> 12)) * ((uint32_t)3125));

  if (_Pressure >= INT32_C(0x40000000))  // Issue #26
    _Pressure = ((_Pressure / (uint32_t)var1) << 1);
  else
    _Pressure = ((_Pressure << 1) / (uint32_t)var1);
  var1 = ((int32_t)_P9 * (int32_t)(((_Pressure >> 3) * (_Pressure >> 3)) >> 13)) >> 12;
  var2 = ((int32_t)(_Pressure >> 2) * (int32_t)_P8) >> 13;
  var3 = ((int32_t)(_Pressure >> 8) * (int32_t)(_Pressure >> 8) * (int32_t)(_Pressure >> 8) *
          (int32_t)_P10) >>
         17;
  _Pressure = (int32_t)(_Pressure) + ((var1 + var2 + var3 + ((int32_t)_P7 << 7)) >> 4);

  // Calculate the humidity
  adc_hum = (uint16_t)(((uint32_t)ucTemp[8] << 8) | (uint32_t)ucTemp[9]);
  temp_scaled = (((int32_t)_tfine * 5) + 128) >> 8;
  var1        = (int32_t)(adc_hum - ((int32_t)((int32_t)_H1 * 16))) -
         (((temp_scaled * (int32_t)_H3) / ((int32_t)100)) >> 1);
  var2 =
      ((int32_t)_H2 *
       (((temp_scaled * (int32_t)_H4) / ((int32_t)100)) +
        (((temp_scaled * ((temp_scaled * (int32_t)_H5) / ((int32_t)100))) >> 6) / ((int32_t)100)) +
        (int32_t)(1 << 14))) >> 10;
  var3      = var1 * var2;
  var4      = (int32_t)_H6 << 7;
  var4      = ((var4) + ((temp_scaled * (int32_t)_H7) / ((int32_t)100))) >> 4;
  var5      = ((var3 >> 14) * (var3 >> 14)) >> 10;
  var6      = (var4 * var5) >> 1;
  _Humidity = (((var3 + var6) >> 10) * ((int32_t)1000)) >> 12;
  if (_Humidity > 100000) /* Cap at 100%rH */
    _Humidity = 100000;
  else if (_Humidity < 0)
    _Humidity = 0;

  Serial.print("Temp = ");
  Serial.println(_Temperature, DEC);
  Serial.print("Humidity = ");
  Serial.println(_Humidity, DEC);
  Serial.print("Pressure = ");
  Serial.println(_Pressure, DEC);
} /* BME680GetReadings() */

void BME680PowerUp(void)
{
  uint8_t uc[2];
  
  GetCalibration();
//  I2CReadRegister(&bbi2c, BME680_ADDR, 0x74, &uc[1], 1); // read CTRL_MEASURE_REG (temp/pressure)
  uc[1] = 1; // start first reading
  uc[1] |= 0x80; // temp oversampling 8x
  uc[1] |= 0x10; // pressure oversampling 8x
  uc[0] = 0x74; // write it back to the same register
  I2CWrite(&bbi2c, BME680_ADDR, uc, 2);
  uc[0] = 0x72; // humidity oversampling control
  uc[1] = 4; // 8x oversampling
  I2CWrite(&bbi2c, BME680_ADDR, uc, 2);
}
void setup() {
  Serial.begin(115200);
  memset(&bbi2c, 0, sizeof(bbi2c));
  bbi2c.bWire = 0; // use bit bang, not wire library
  bbi2c.iSDA = SDA_PIN;
  bbi2c.iSCL = SCL_PIN;
  I2CInit(&bbi2c, 100000L);
  delay(100); // allow devices to power up
  BME680PowerUp();
  delay(100);
}

void loop() {
  BME680GetReadings();
  delay(1000);
}
