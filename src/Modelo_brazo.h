// ##########################################################################
// >>> ARCHIVO 3: Modelo_Brazo (.h y .cpp)
// ##########################################################################
#pragma once
#include "config.h"
#include "Modelo_cinematica.h"

// Estado del brazo: angulo actual de cada servo (grados)
struct Pose {
    float angulos[CANTIDAD_SERVOS];
};

Pose poseActual;
int cubosApilados = 0;          // Cubos que ya hay en la pila (define la altura del siguiente)
bool detenerRutina = false;     // Se activa con el comando 's' (STOP)


// Estado que se informa a la app (mensajes STAT y RUT)
const char* estadoActual = "IDLE";   // IDLE, RUTINA, HOME, MANUAL, DETENIDO, ERROR
int posicionActual = 0;              // Posicion que se esta revisando (1 a CANTIDAD_POSICIONES); 0 = ninguna
float ultimaDistanciaCm = -1.0f;     // Ultima lectura del sensor (-1 = sin lectura)

/**
 * @brief Posicion actual del punto de agarre, calculada con la cinematica directa
 *        a partir de los angulos de servo. Requiere la calibracion.
 */
bool obtenerPuntoActual(Punto& punto) {
    float q[CANTIDAD_ARTICULACIONES];
    for (int j = 0; j < CANTIDAD_ARTICULACIONES; j++) {
        if (!definido(SERVO_OFFSET[j]) || !definido(SERVO_SIGNO[j])) return false;
        q[j] = servoAArticulacion(j, poseActual.angulos[j]);
    }
    punto = posicionDe(cinematicaDirecta(q));
    return true;
}

