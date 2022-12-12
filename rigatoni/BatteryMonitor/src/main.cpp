/*
 * Max capacity of one battery is 35 Ah. Total max capacity of 4 batteries is 140 Ah.
 * Rated to discharge at 3.5 A over 10 hours at 1.75V/cell (10.5V total)
 * SLA1156 Interstate battery has 6 cells.
 * Internal resistance is 9.5mOhms (too low to affect voltage reading since high value resistors are used)
 * 
 * 3/13 Battery measurement: 49.34V
 * 3/28 Battery measurement: 49.34 V = 26.2% SOC = 36.71 Ah
 * 
 * TODO: Experimentally measure the current max capacity and voltage during discharge of the batteries.
 *       Create communication link between battery monitor and neopixel board.
 *       Display state of charge onto LCD
 * 
 * Coulomb Counting Equation: SOC(t) = 1/C * integral_0_to_t(I(p)*dp)
 */
#include <mbed.h>

Ticker accumulator;
FlashIAP internal_flash;
AnalogIn volt_pin(A5);
AnalogIn current_pin(A0);
float state_of_charge;
double charge_consumed = 0;
double time_interval = 0.01;

/*
 * Q (charge in coulombs) = I * t
 * 0 - 3.3V output of current sensor translates to -200A to +200A input, 1.65V corresponds to 0A
 * Try to find another point for the current sensor to find an accurate slope
 * (0, 1.65), (200, 3.3)
 */
void update_current_draw() {
  double current = 200 / 1.65 * current_pin.read_voltage() + 1.65;
  charge_consumed += current * time_interval;
}

/*
 * Open Circuit Voltage vs Remaining Capacity graph on page 2: https://drive.google.com/file/d/1tNnYurx3ijrAbbN9-7NRjk-6qulW955e/view
 * Top of range: (13, 100), (11.8, 0)
 * Bottom of range: (12.75, 100), (11.25, 0)
 * y is state of charge, x is voltage
 * y = 2000/27 * x - 23050/27
 * 
 * 51.5V is 100% SOC
 * 46.1V is 0% SOC
 *
double calculate_datasheet_state_of_charge(double voltage) {
  return 2000.0 / 108 * voltage - 23050.0 / 27;
}*/

/*
 * 56V is 100% SOC or 3.027027027...V from pin
 * 47V is 0% SOC or 2.540540540...V from pin
 * Voltage divider ratio is 2/37 (from voltage source to output), so getting battery voltage from the voltage reading on the pin is found by
 * multiplying by 37/2
 */
double calculate_state_of_charge(double pin_voltage) {
  return 100.0 / 9 * 37 / 2 * pin_voltage - 4700.0 / 9;
}

int main() {
  float volt_data = 0;
  float current_data = 0;
  time_t seconds = time(NULL);
  ADC_HandleTypeDef g_AdcHandle;
  g_AdcHandle.Instance = ADC1;
  g_AdcHandle.Init.Resolution = ADC_RESOLUTION_16B;
  HAL_ADC_Init(&g_AdcHandle);
  state_of_charge = calculate_state_of_charge(volt_pin.read()); // update state of charge from battery voltage
  accumulator.attach(&update_current_draw, 1ms);
  
  while (1) {
    volt_data = volt_pin.read_voltage();
    current_data = current_pin.read_voltage();
    printf("%.15f\n", calculate_state_of_charge(2.667027027027027));
    printf("%.15f\n", calculate_state_of_charge(2.809837837837837));
    /* Actual: 26
               55.355555555555384
    */
    //printf("%f\n", volt_pin.get_reference_voltage());
    /*printf("voltage: %d\n", (int) (current_data * 1000));
    printf("current: %d\n", (int) (volt_data * 1000));
    printf("Time as a basic string = %s\n", ctime(&seconds));*/
    ThisThread::sleep_for(5s);
  }
}
