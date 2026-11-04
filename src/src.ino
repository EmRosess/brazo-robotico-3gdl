
// ##########################################################################
// >>> ARCHIVO 9: PickAndPlace.ino
// ##########################################################################
#include "config.h"
#include "Driver_servos.h"
#include "Driver_sensor.h"
#include "Modelo_cinematica.h"
#include "Controlador_comandos.h"
#include "Modelo_brazo.h"
#include "Vista_telemetria.h"


void setup() {
    vistaIniciar();
    iniciarSensor();
    iniciarServos();
   autopruebaCinematica();

   if (configuracionLista()) {
        Serial.println("[OK] Configuracion completa.");
    }
    mostrarAyuda();
    
}

void loop() {
    procesarPuertoSerie();
}
