/**
 * @file driver_wifi.h
 * @brief Enlace con la app: conexion WiFi de la ESP32 y servidor TCP que repite el
 *        protocolo del Monitor Serie.
 * @author Emilio Rojas
 *
 * "Enlace" se usa igual que Serial (print, println, available, readStringUntil), pero:
 *   - lo que se imprime sale por el Monitor Serie y tambien por WiFi (si hay una app conectada);
 *   - las ordenes pueden llegar por el Monitor Serie o por WiFi.
 * Asi toda la logica del brazo queda igual: solo cambia el medio de comunicacion.
 *
 * Este modulo no conoce servos, cinematica ni rutinas: solo transporta lineas de texto.
 * Las ordenes que envie la app deben terminar en '\n'.
 */

#pragma once

#include <Arduino.h>
#include <WiFi.h>

class EnlaceApp : public Stream {
public:
    /**
     * @brief Conecta a la red WiFi; si no puede, crea un punto de acceso propio. Luego abre el
     *        servidor TCP. Debe llamarse despues de Serial.begin().
     * @param ssid, clave Red a la que conectarse.
     * @param apSsid, apClave Red propia de respaldo (clave de 8 o mas caracteres).
     * @param puerto Puerto TCP donde escucha el robot.
     * @param timeoutMs Tiempo maximo de espera para conectar a la red.
     */
    void iniciar(const char* ssid, const char* clave,
                 const char* apSsid, const char* apClave,
                 uint16_t puerto, unsigned long timeoutMs);

    /** @brief true si hay una app conectada por WiFi. */
    bool appConectada();

    /** @brief Direccion IP del robot (la que se usa en la app). */
    String direccionIp();

    // ---- Interfaz Stream (lo que usa el resto del codigo) ----
    int available() override;
    int read() override;
    int peek() override;
    size_t write(uint8_t dato) override;
    size_t write(const uint8_t* datos, size_t cantidad) override;
    void flush() override;

private:
    void atenderConexiones();   // acepta una app nueva y detecta si se desconecto
    void enviarPendiente();     // manda por WiFi lo acumulado en el bufer de salida
};

/** Unica instancia: se usa en lugar de Serial. */
extern EnlaceApp Enlace;
