void check() {
    const int R1 = 120;
    const int R2 = 36;
    const int PIN_VBAT = A0;
    float volts = 0;
    int adc = analogRead(PIN_VBAT);
    
    if (adc > 3000) {
        volts = 0.0005 * adc + 1.0874;
    } else {
        volts = 0.0008 * adc + 0.1372;
    }
    
    float battery_voltage = volts * (R1 + R2) / R2;
    
    Serial.println("Voltage Check");
    Serial.print("ADC Value: "); Serial.println(adc);
    Serial.print("Voltage: "); Serial.println(volts);
    Serial.print("Battery Voltage: "); Serial.println(battery_voltage);
    Serial.println(); // Adds a blank line for better readability
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting...");
}

void loop() {
    check();
    delay(2000);
}
