
// ##########################################################################
// >>> ARCHIVO 4: Driver_Servo (.h y .cpp)
// ##########################################################################
#pragma once
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include "Modelo_brazo.h"

// Instancia del controlador PCA9685
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(DIRECCION_PCA9685);

/**
 * @brief Traduce grados (0-180) a pulsos de ciclo de trabajo del PCA9685.
 */
int convertirAnguloAPwm(float angulo) {
    float anguloRestringido = constrain(angulo, 0.0f, 180.0f);
    return PULSO_MINIMO + (int)lroundf(anguloRestringido / 180.0f * (PULSO_MAXIMO - PULSO_MINIMO));
}

/**
 * @brief Envia la senal PWM correspondiente al angulo para un solo canal.
 */
void posicionarServo(int canal, float angulo) {
    pca9685.setPWM(canal, 0, convertirAnguloAPwm(angulo));
}

/**
 * @brief Inicializa el bus I2C y el PCA9685, y lleva los 4 servos al angulo inicial.
 */
void iniciarServos() {
    Wire.begin(PIN_SDA, PIN_SCL);

    // Comprobar que el PCA9685 responde en el bus I2C
    Wire.beginTransmission(DIRECCION_PCA9685);
    if (Wire.endTransmission() != 0) {
        Enlace.println("[ERROR] El PCA9685 no responde. Revisa SDA, SCL, VCC y GND.");
    }

    pca9685.begin();
    pca9685.setPWMFreq(FRECUENCIA_PWM);

    // Posicion inicial de seguridad
    for (int canal = 0; canal < CANTIDAD_SERVOS; canal++) {
        poseActual.angulos[canal] = ANGULO_INICIAL;
        posicionarServo(canal, ANGULO_INICIAL);
        delay(100);   // Pausa escalonada de arranque: evita picos de corriente simultaneos
    }
}

// El driver no conoce la app: en cada paso llama a esta funcion (si existe), que atiende
// la telemetria y el STOP. Se asigna en setup().
FuncionDeServicio servicioDeFondo = nullptr;

/**
 * @brief Atiende la comunicacion y devuelve true si hay que detener el movimiento (STOP).
 */
bool paradaSolicitada() {
    if (servicioDeFondo != nullptr) servicioDeFondo();
    return detenerRutina;
}

/**
 * @brief Espera ms milisegundos sin dejar de atender la app (telemetria y STOP).
 */
void esperarMs(unsigned long ms) {
    unsigned long inicio = millis();
    while (millis() - inicio < ms) {
        if (paradaSolicitada()) return;
        delay(5);
    }
}

/**
 * @brief Mueve un servo de forma gradual (1 grado por paso) hasta el destino, sin afectar a los demas.
 */
void moverServoSuavemente(int canal, float destino, int pausaMs) {
    if (detenerRutina) return;
    destino = constrain(destino, 0.0f, 180.0f);

    while (fabsf(destino - poseActual.angulos[canal]) > 0.001f) {
        if (paradaSolicitada()) return;

        float diferencia = destino - poseActual.angulos[canal];
        float paso = (fabsf(diferencia) < 1.0f) ? diferencia : (diferencia > 0.0f ? 1.0f : -1.0f);

        poseActual.angulos[canal] += paso;
        posicionarServo(canal, poseActual.angulos[canal]);
        delay(pausaMs);
        yield();   // Evita que el Watchdog Timer reinicie la ESP32 en movimientos largos
    }
}

/**
 * @brief Mueve varios servos a la vez; todos llegan juntos (el que mas recorre avanza 1 grado por paso).
 * @param destino Angulos finales de los CANTIDAD_SERVOS servos.
 */
void moverServosSincronizados(const float destino[], int pausaMs) {
    if (detenerRutina) return;

    float inicio[CANTIDAD_SERVOS];
    float fin[CANTIDAD_SERVOS];
    float mayorRecorrido = 0.0f;
    for (int i = 0; i < CANTIDAD_SERVOS; i++) {
        inicio[i] = poseActual.angulos[i];
        fin[i] = constrain(destino[i], 0.0f, 180.0f);
        mayorRecorrido = fmaxf(mayorRecorrido, fabsf(fin[i] - inicio[i]));
    }

    int pasos = (int)ceilf(mayorRecorrido);
    if (pasos < 1) pasos = 1;

    for (int p = 1; p <= pasos; p++) {
        if (paradaSolicitada()) return;

        float fraccion = (float)p / pasos;
        for (int i = 0; i < CANTIDAD_SERVOS; i++) {
            poseActual.angulos[i] = inicio[i] + (fin[i] - inicio[i]) * fraccion;
            posicionarServo(i, poseActual.angulos[i]);
        }
        delay(pausaMs);
        yield();
    }
}
