
// ##########################################################################
// >>> ARCHIVO 8: Controlador_Comandos (.h y .cpp)
// ##########################################################################
#pragma once
#include "Controlador_rutina.h"

void procesarPuertoSerie() {
    if (Serial.available() > 0) {
        String entrada = Serial.readStringUntil('\n');
        entrada.trim();
        entrada.toLowerCase();
        detenerRutina = false;   // Cada orden nueva parte sin STOP activo

        if (entrada.length() == 0) return;

        // 1. Comandos de una letra
        if (entrada == "i") {
            ejecutarRutinaApilado();
            return;
        }
        if (entrada == "h") {
            volverAInicio();
            return;
        }
        if (entrada == "f") {
            mostrarEstado();
            return;
        }
        if (entrada == "c") {
            if (configuracionLista()) validarRutina();
            return;
        }
        if (entrada == "d") {
            float distancia = medirDistanciaCm();
            if (distancia > 0.0f) {
                Serial.print("[INFO] Distancia: ");
                Serial.print(distancia, 1);
                Serial.print(" cm -> ");
                Serial.println(hayObjeto(distancia) ? "OBJETO DETECTADO" : "vacio");
            }
            return;
        }
        if (entrada == "s") {
            Serial.println("[INFO] No hay ningun movimiento en curso.");
            return;
        }

        // 2. Ir a un punto: g x y z
        if (entrada.startsWith("g")) {
            Punto destino;
            if (sscanf(entrada.c_str(), "g %f %f %f", &destino.x, &destino.y, &destino.z) == 3) {
                moverAPuntoArticular(destino);
            } else {
                Serial.println("[AVISO] Formato invalido. Usa: g x y z   (ej: g 110 -50 85)");
            }
            return;
        }

        // 3. Control manual: <servo> <angulo>
        int canal;
        float angulo;
        if (sscanf(entrada.c_str(), "%d %f", &canal, &angulo) != 2) {
            Serial.println("[AVISO] Comando invalido. Escribe 'i', 'h', 'd', 'f', 'c', 'g x y z' o '<servo> <angulo>'");
            return;
        }
        if (canal < 0 || canal >= CANTIDAD_SERVOS) {
            Serial.println("[ERROR] Servo fuera de rango (0-3).");
            return;
        }
        if (angulo < 0.0f || angulo > 180.0f) {
            Serial.println("[ERROR] Angulo fuera de rango (0-180).");
            return;
        }

        Serial.print("[INFO] Moviendo Servo ");
        Serial.print(canal);
        Serial.print(" a ");
        Serial.println(angulo, 1);
        moverServoSuavemente(canal, angulo, PAUSA_PASO_ARTICULAR_MS);
    }
}

