
// ##########################################################################
// >>> ARCHIVO 5: Driver_Sensor (.h y .cpp)
// ##########################################################################
#pragma once
#include "config.h"
void iniciarSensor() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);
}

/**
 * @brief Mide la distancia con el HC-SR04 y devuelve la mediana de varias lecturas.
 * @return Distancia en cm, o -1 si el sensor no entrego lecturas validas.
 */
float medirDistanciaCm() {
    float muestras[MUESTRAS_SENSOR];
    int validas = 0;

    for (int i = 0; i < MUESTRAS_SENSOR; i++) {
        // Pulso de disparo de 10 microsegundos
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);

        unsigned long duracion = pulseIn(ECHO_PIN, HIGH, TIMEOUT_ECO_US);
        if (duracion > 0) {
            muestras[validas] = duracion * 0.0343f / 2.0f;   // velocidad del sonido en cm/us, ida y vuelta
            validas++;
        }
        delay(PAUSA_ENTRE_MUESTRAS_MS);
    }

    if (validas < 3) {
        Enlace.println("[AVISO] Sensor sin lecturas validas: revisa conexion y divisor de tension.");
        return -1.0f;
    }

    // Ordenar (insercion) y devolver la mediana
    for (int i = 1; i < validas; i++) {
        float valor = muestras[i];
        int j = i - 1;
        while (j >= 0 && muestras[j] > valor) {
            muestras[j + 1] = muestras[j];
            j--;
        }
        muestras[j + 1] = valor;
    }
    return muestras[validas / 2];
}