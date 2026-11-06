
// ##########################################################################
// >>> ARCHIVO 8: Controlador_Comandos (.h y .cpp)
// ##########################################################################
#pragma once
#include "Controlador_rutina.h"
#include "Modelo_cinematica.h"
/**
 * @brief Servicio durante un movimiento: envia la telemetria periodica y atiende STOP.
 *        Mientras el brazo se mueve, cualquier otro comando se rechaza con ERR,OCUPADO.
 */
void servicioDuranteMovimiento() {
    emitirTelemetriaPeriodica();

    if (Enlace.available() > 0) {
        String linea = Enlace.readStringUntil('\n');
        linea.trim();
        linea.toUpperCase();
        if (linea == "STOP" || linea == "S") {
            detenerRutina = true;
            Enlace.println("[AVISO] STOP: movimiento detenido por el usuario.");
            responderOK("STOP");
        } else if (linea.length() > 0) {
            responderError("OCUPADO");
        }
    }
}

// ---- Acciones (las usan tanto la app como el Monitor Serie) ----

void comandoHome() {
    estadoActual = "HOME";
    emitirTelemetria();
    volverAInicio();

    if (detenerRutina) {
        estadoActual = "DETENIDO";
        responderError("DETENIDA");
    } else {
        estadoActual = "IDLE";
        responderOK("HOME");
    }
    emitirTelemetria();
}

void comandoAuto() {
    estadoActual = "RUTINA";
    emitirTelemetria();

    ResultadoRutina resultado = ejecutarRutinaApilado();
    posicionActual = 0;

    switch (resultado) {
        case RUTINA_OK:
            estadoActual = "IDLE";
            Enlace.print("OK,AUTO,");
            Enlace.println(cubosApilados);
            break;
        case RUTINA_DETENIDA:
            estadoActual = "DETENIDO";
            responderError("DETENIDA");
            break;
        case RUTINA_CONFIG:
            estadoActual = "IDLE";
            responderError("CONFIG");
            break;
        case RUTINA_ALCANCE:
            estadoActual = "IDLE";
            responderError("ALCANCE");
            break;
        case RUTINA_ERROR:
            estadoActual = "ERROR";
            responderError("MOVIMIENTO");
            break;
    }
    emitirTelemetria();
}

/**
 * @brief Control manual de un servo desde la app. Respeta el rango seguro de cada servo.
 */
void comandoServo(int canal, int angulo) {
    if (canal < 0 || canal >= CANTIDAD_SERVOS) {
        responderError("CANAL");
        return;
    }

    int minimo = (canal == PINZA) ? LIMITE_PINZA_MIN : (int)LIMITE_SERVO_MIN[canal];
    int maximo = (canal == PINZA) ? LIMITE_PINZA_MAX : (int)LIMITE_SERVO_MAX[canal];
    if (angulo < minimo || angulo > maximo) {
        Enlace.print("ERR,RANGO,");
        Enlace.print(canal);
        Enlace.print(",");
        Enlace.print(minimo);
        Enlace.print("-");
        Enlace.println(maximo);
        return;
    }

    estadoActual = "MANUAL";
    emitirTelemetria();
    moverServoSuavemente(canal, angulo, PAUSA_PASO_ARTICULAR_MS);

    if (detenerRutina) {
        estadoActual = "DETENIDO";
        responderError("DETENIDA");
    } else {
        estadoActual = "IDLE";
        Enlace.print("OK,S,");
        Enlace.print(canal);
        Enlace.print(",");
        Enlace.println(angulo);
    }
    emitirTelemetria();
}

// ---- Lectura de comandos ----

void procesarPuertoSerie() {
    if (Enlace.available() <= 0) return;

    String entrada = Enlace.readStringUntil('\n');
    entrada.trim();
    if (entrada.length() == 0) return;
    detenerRutina = false;   // Cada orden nueva parte sin STOP activo

    // 1. Protocolo de la app: HOME, AUTO, STOP, S,<canal>,<angulo>
    String mayusculas = entrada;
    mayusculas.toUpperCase();

    if (mayusculas == "HOME") {
        comandoHome();
        return;
    }
    if (mayusculas == "AUTO") {
        comandoAuto();
        return;
    }
    if (mayusculas == "STOP") {
        responderOK("STOP");
        return;
    }
    if (mayusculas.startsWith("S,")) {
        int canal;
        int angulo;
        if (sscanf(mayusculas.c_str(), "S,%d,%d", &canal, &angulo) == 2) {
            comandoServo(canal, angulo);
        } else {
            responderError("FORMATO");
        }
        return;
    }

    // 2. Comandos del Monitor Serie (calibracion y pruebas)
    entrada.toLowerCase();

    if (entrada == "i") {
        comandoAuto();
        return;
    }
    if (entrada == "h") {
        comandoHome();
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
        ultimaDistanciaCm = distancia;
        if (distancia > 0.0f) {
            Enlace.print("[INFO] Distancia: ");
            Enlace.print(distancia, 1);
            Enlace.print(" cm -> ");
            Enlace.println(hayObjeto(distancia) ? "OBJETO DETECTADO" : "vacio");
        }
        return;
    }
    if (entrada == "s") {
        Enlace.println("[INFO] No hay ningun movimiento en curso.");
        return;
    }

    // Ir a un punto: g x y z
    if (entrada.startsWith("g")) {
        Punto destino;
        if (sscanf(entrada.c_str(), "g %f %f %f", &destino.x, &destino.y, &destino.z) == 3) {
            estadoActual = "MANUAL";
            moverAPuntoArticular(destino);
            estadoActual = detenerRutina ? "DETENIDO" : "IDLE";
        } else {
            Enlace.println("[AVISO] Formato invalido. Usa: g x y z   (ej: g 110 -50 85)");
        }
        return;
    }

    // Control manual sin restricciones (0-180), para calibrar: <servo> <angulo>
    int canal;
    float angulo;
    if (sscanf(entrada.c_str(), "%d %f", &canal, &angulo) != 2) {
        responderError("COMANDO");
        Enlace.println("[AVISO] Comando invalido. App: HOME, AUTO, STOP, S,<canal>,<angulo>. Monitor Serie: 'i', 'h', 'd', 'f', 'c', 'g x y z' o '<servo> <angulo>'");
        return;
    }
    if (canal < 0 || canal >= CANTIDAD_SERVOS) {
        Enlace.println("[ERROR] Servo fuera de rango (0-3).");
        return;
    }
    if (angulo < 0.0f || angulo > 180.0f) {
        Enlace.println("[ERROR] Angulo fuera de rango (0-180).");
        return;
    }

    Enlace.print("[INFO] Moviendo Servo ");
    Enlace.print(canal);
    Enlace.print(" a ");
    Enlace.println(angulo, 1);
    estadoActual = "MANUAL";
    moverServoSuavemente(canal, angulo, PAUSA_PASO_ARTICULAR_MS);
    estadoActual = detenerRutina ? "DETENIDO" : "IDLE";
}
