
// ##########################################################################
// >>> ARCHIVO 6: Vista_Telemetria (.h y .cpp)
// ##########################################################################
#pragma once
#include "Modelo_brazo.h"
void mostrarPunto(const char* nombre, const Punto& p) {
    Serial.print(nombre);
    Serial.print(": x=");
    Serial.print(p.x, 1);
    Serial.print("  y=");
    Serial.print(p.y, 1);
    Serial.print("  z=");
    Serial.print(p.z, 1);
    Serial.println(" mm");
}

void mostrarAyuda() {
    Serial.println("\nComandos:");
    Serial.println("  'i'            -> Ejecutar la rutina de apilado");
    Serial.println("  'h'            -> Volver a la posicion inicial");
    Serial.println("  'd'            -> Medir la distancia del sensor (calibracion del umbral)");
    Serial.println("  'f'            -> Estado: angulos, q y posiciones D-H de las articulaciones");
    Serial.println("  'c'            -> Comprobar configuracion y alcance de la rutina");
    Serial.println("  'g x y z'      -> Mover la garra al punto (mm), ej: g 110 -50 85");
    Serial.println("  's'            -> STOP (durante un movimiento)");
    Serial.println("  '<servo> <angulo>' -> Control manual (servo 0-3, angulo 0-180)");
}

void vistaIniciar() {
    Serial.begin(115200);

    // Espera a que el monitor serie este listo, con un maximo de 2 s
    unsigned long tiempoInicio = millis();
    while (!Serial && (millis() - tiempoInicio < 2000));

    Serial.println("\n========================================");
    Serial.println("  BRAZO ROBOTICO: APILADO CON D-H (ESP32)");
    Serial.println("========================================");
}

/**
 * @brief Muestra los angulos de servo y, si hay calibracion, q y las posiciones D-H de las articulaciones.
 */
void mostrarEstado() {
    Serial.println("\n--- ESTADO DEL BRAZO ---");
    Serial.print("Servos [BASE HOMBRO CODO PINZA]: ");
    for (int i = 0; i < CANTIDAD_SERVOS; i++) {
        Serial.print(poseActual.angulos[i], 1);
        Serial.print(" ");
    }
    Serial.println();

    float q[CANTIDAD_ARTICULACIONES];
    for (int j = 0; j < CANTIDAD_ARTICULACIONES; j++) {
        if (!definido(SERVO_OFFSET[j]) || !definido(SERVO_SIGNO[j])) {
            Serial.println("[AVISO] Falta la calibracion (SERVO_OFFSET / SERVO_SIGNO): no se puede calcular la cinematica.");
            return;
        }
        q[j] = servoAArticulacion(j, poseActual.angulos[j]);
    }

    Serial.print("q1, q2, q3 (grados): ");
    Serial.print(q[BASE], 1);
    Serial.print(", ");
    Serial.print(q[HOMBRO], 1);
    Serial.print(", ");
    Serial.println(q[CODO], 1);

    Matriz4 T[5];
    cadenaDH(q, T);
    const char* nombres[5] = {"Base  ", "Hombro", "Codo  ", "Muneca", "Garra "};
    for (int i = 0; i < 5; i++) {
        mostrarPunto(nombres[i], posicionDe(T[i]));
    }
}

