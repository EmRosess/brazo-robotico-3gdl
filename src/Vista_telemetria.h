
// ##########################################################################
// >>> ARCHIVO 6: Vista_Telemetria (.h y .cpp)
// ##########################################################################
#pragma once
#include "Modelo_brazo.h"

void mostrarPunto(const char* nombre, const Punto& p) {
    Enlace.print(nombre);
    Enlace.print(": x=");
    Enlace.print(p.x, 1);
    Enlace.print("  y=");
    Enlace.print(p.y, 1);
    Enlace.print("  z=");
    Enlace.print(p.z, 1);
    Enlace.println(" mm");
}

void mostrarAyuda() {
    Enlace.print("\nApp por WiFi: TCP a ");
    Enlace.print(Enlace.direccionIp());
    Enlace.print(":");
    Enlace.println(WIFI_PUERTO);
    Enlace.println("Comandos de la app: HOME, AUTO, STOP, S,<canal>,<angulo>");
    Enlace.println("Comandos del Monitor Serie:");
    Enlace.println("  'i'            -> Ejecutar la rutina de apilado");
    Enlace.println("  'h'            -> Volver a la posicion inicial");
    Enlace.println("  'd'            -> Medir la distancia del sensor (calibracion del umbral)");
    Enlace.println("  'f'            -> Estado: angulos, q y posiciones D-H de las articulaciones");
    Enlace.println("  'c'            -> Comprobar configuracion y alcance de la rutina");
    Enlace.println("  'g x y z'      -> Mover la garra al punto (mm), ej: g 110 -50 85");
    Enlace.println("  's'            -> STOP (durante un movimiento; la app envia STOP)");
    Enlace.println("  '<servo> <angulo>' -> Control manual (servo 0-3, angulo 0-180)");
}

void vistaIniciar() {
    Serial.begin(BAUDIOS_SERIE);
    Enlace.setTimeout(50);   // Las lineas se leen rapido: no bloquea si llega un comando incompleto

    // Espera a que el monitor serie este listo, con un maximo de 2 s
    unsigned long tiempoInicio = millis();
    while (!Serial && (millis() - tiempoInicio < 2000));

    Enlace.println("\n========================================");
    Enlace.println("  BRAZO ROBOTICO: APILADO CON D-H (ESP32)");
    Enlace.println("========================================");
}

/**
 * @brief Muestra los angulos de servo y, si hay calibracion, q y las posiciones D-H de las articulaciones.
 */
void mostrarEstado() {
    Enlace.println("\n--- ESTADO DEL BRAZO ---");
    Enlace.print("Servos [BASE HOMBRO CODO PINZA]: ");
    for (int i = 0; i < CANTIDAD_SERVOS; i++) {
        Enlace.print(poseActual.angulos[i], 1);
        Enlace.print(" ");
    }
    Enlace.println();

    float q[CANTIDAD_ARTICULACIONES];
    for (int j = 0; j < CANTIDAD_ARTICULACIONES; j++) {
        if (!definido(SERVO_OFFSET[j]) || !definido(SERVO_SIGNO[j])) {
            Enlace.println("[AVISO] Falta la calibracion (SERVO_OFFSET / SERVO_SIGNO): no se puede calcular la cinematica.");
            return;
        }
        q[j] = servoAArticulacion(j, poseActual.angulos[j]);
    }

    Enlace.print("q1, q2, q3 (grados): ");
    Enlace.print(q[BASE], 1);
    Enlace.print(", ");
    Enlace.print(q[HOMBRO], 1);
    Enlace.print(", ");
    Enlace.println(q[CODO], 1);

    Matriz4 T[5];
    cadenaDH(q, T);
    const char* nombres[5] = {"Base  ", "Hombro", "Codo  ", "Muneca", "Garra "};
    for (int i = 0; i < 5; i++) {
        mostrarPunto(nombres[i], posicionDe(T[i]));
    }
}


// ---- Protocolo hacia la app ----
//   STAT,<estado>,<base>,<hombro>,<codo>,<pinza>        estado y angulos de servo (enteros)
//   DH,<x1>,<y1>,<z1>,<x2>,<y2>,<z2>,<x3>,<y3>,<z3>,<x4>,<y4>,<z4>   hombro, codo, muneca y garra (mm, D-H)
//   RUT,<posicion>,<cubos>,<distancia_cm>                progreso de la rutina y ultima lectura del sensor
//   OK,<...>  /  ERR,<motivo>                            respuesta a cada comando

unsigned long ultimaTelemetriaMs = 0;

void responderOK(const char* texto) {
    Enlace.print("OK,");
    Enlace.println(texto);
}

void responderError(const char* motivo) {
    Enlace.print("ERR,");
    Enlace.println(motivo);
}

/**
 * @brief Envia a la app el estado actual: STAT siempre; DH si hay calibracion; RUT con el progreso.
 */
void emitirTelemetria() {
    Enlace.print("STAT,");
    Enlace.print(estadoActual);
    for (int i = 0; i < CANTIDAD_SERVOS; i++) {
        Enlace.print(",");
        Enlace.print((int)lroundf(poseActual.angulos[i]));
    }
    Enlace.println();

    // Posiciones D-H de las articulaciones (necesitan la calibracion)
    float q[CANTIDAD_ARTICULACIONES];
    bool calibrado = true;
    for (int j = 0; j < CANTIDAD_ARTICULACIONES; j++) {
        if (!definido(SERVO_OFFSET[j]) || !definido(SERVO_SIGNO[j])) {
            calibrado = false;
            break;
        }
        q[j] = servoAArticulacion(j, poseActual.angulos[j]);
    }
    if (calibrado) {
        Matriz4 T[5];
        cadenaDH(q, T);
        Enlace.print("DH");
        for (int i = 1; i < 5; i++) {
            Punto p = posicionDe(T[i]);
            Enlace.print(",");
            Enlace.print(p.x, 1);
            Enlace.print(",");
            Enlace.print(p.y, 1);
            Enlace.print(",");
            Enlace.print(p.z, 1);
        }
        Enlace.println();
    }

    Enlace.print("RUT,");
    Enlace.print(posicionActual);
    Enlace.print(",");
    Enlace.print(cubosApilados);
    Enlace.print(",");
    Enlace.println(ultimaDistanciaCm, 1);
}

/**
 * @brief Envia la telemetria si ya paso TELEMETRIA_MS desde la ultima vez.
 */
void emitirTelemetriaPeriodica() {
    unsigned long ahora = millis();
    if (ahora - ultimaTelemetriaMs >= TELEMETRIA_MS) {
        ultimaTelemetriaMs = ahora;
        emitirTelemetria();
    }
}

