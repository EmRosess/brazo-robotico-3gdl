#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Instancia del controlador PCA9685
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(DIRECCION_PCA9685);


// ==========================================
// MÓDULOS DE INICIALIZACIÓN
// ==========================================

/**
 * @brief Inicializa los buses de comunicación (I2C y Serial) y lleva el brazo a la pose inicial.
 */
void inicializarSistema() {
    Serial.begin(115200);
    while (!Serial); // Espera opcional a que el monitor serie esté listo

    Wire.begin(PIN_SDA, PIN_SCL);

    pca9685.begin();
    pca9685.setPWMFreq(FRECUENCIA_PWM);

    // Posición inicial de seguridad
    for (int canal = 0; canal < CANTIDAD_SERVOS; canal++) {
        posicionarServo(canal, poseActual.angulos[canal]);
    }

    Serial.println("\n========================================");
    Serial.println("  BRAZO ROBOTICO: RUTINA PICK AND PLACE ");
    Serial.println("========================================");
    Serial.println("Comandos:  i = ejecutar rutina | h = volver a inicio");
}