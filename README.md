# Brazo robótico apilador de cubos (ESP32)

Brazo robótico de 3 grados de libertad más pinza, controlado por una ESP32. Revisa tres posiciones con un sensor ultrasónico, toma los cubos que encuentra y los apila en una pila fija. Se puede manejar por el Monitor Serie, por WiFi desde una app Android, o con un panel en Python (Streamlit).

> Proyecto académico. Autores: Emilio Rojas (USACH), Martin Barrezueta (USACH), Ignacio Espinoza (USACH).

## Características

- Cinemática directa e inversa analítica con parámetros Denavit-Hartenberg (D-H).
- Rutina automática: escaneo de 3 posiciones, recogida y apilado en 3 niveles.
- Trayectorias suaves (interpolación polar y perfil de coseno) con validación previa de alcance y límites de servo.
- Control de cada servo por separado y parada de emergencia (`STOP`) durante cualquier movimiento.
- Telemetría periódica (estado, ángulos, posiciones D-H y datos de la rutina).
- Enlace WiFi (servidor TCP) con punto de acceso de respaldo si no hay red.
- Código modular y con límites de responsabilidad claros.

## Hardware

| Componente | Detalle |
| --- | --- |
| Microcontrolador | ESP32 DevKit |
| Driver de servos | PCA9685 (I2C, 50 Hz) |
| Servos | 2 × 9 g (base, pinza), 2 × MG90S  (hombro, codo) |
| Sensor | HC-SR04 (distancia) |
| Alimentación | Cargador de celular 5 V-3A máx con capacitor de 470 µF; GND común con la ESP32 |

Conexiones:

| Señal | Pin ESP32 |
| --- | --- |
| SDA (PCA9685) | 21 |
| SCL (PCA9685) | 22 |
| TRIG (HC-SR04) | 18 |
| ECHO (HC-SR04) | 33 (con divisor de tensión: ECHO entrega 5 V) |

Canales del PCA9685: 0 base, 1 hombro, 2 codo, 3 pinza.

## Estructura del proyecto

```
src/
├── src.ino                 Programa principal (setup / loop)
├── config.h                Constantes: pines, medidas, calibración, rutina, WiFi
├── Modelo_cinematica.h     Matemática D-H: cinemática directa e inversa (funciones puras)
├── Modelo_brazo.h          Estado del brazo (pose actual, cubos apilados)
├── Driver_servos.h         Ángulo en grados -> PWM del PCA9685
├── Driver_sensor.h         Lectura del HC-SR04 (mediana de varias muestras)
├── Driver_wifi.h / .cpp    Enlace Serial + WiFi (TCP)
├── Controlador_rutina.h    Rutina de escaneo y apilado, trayectorias
├── Controlador_comandos.h  Interpreta las órdenes (HOME, AUTO, STOP, S,...)
└── Vista_telemetria.h      Mensajes STAT / DH / RUT hacia la app
app_android/                App Android (Kotlin)
└── MainActivity.kt, ClienteBrazo.kt
docs/                       Informe
```


## Instalación del firmware

1. Instala el Arduino IDE y el soporte de placas **ESP32** (núcleo 3.x).
2. Instala la librería **Adafruit PWM Servo Driver** (PCA9685).
3. Abre `src/src.ino`.
4. En `config.h` completa lo marcado como `PENDIENTE` o `AJUSTAR`:
   - `WIFI_SSID` y `WIFI_CLAVE` con los datos de tu red.
   - `SERVO_OFFSET` y `SERVO_SIGNO` (calibración, ver más abajo).
   - Posiciones de los cubos (`POSICION_X/Y`), de la pila (`PILA_X/Y`) y tamaño del cubo (`DIMENSION_CUBO`).
5. Selecciona la placa ESP32 y el puerto, y sube el programa.
6. Abre el Monitor Serie a **115200 baudios**. Verás la IP del robot.

## Calibración

1. Verifica el montaje de cada servo moviéndolo individualmente (`S,<canal>,<ángulo>`).
2. Define `SERVO_OFFSET` (ángulo de servo cuando q = 0) y `SERVO_SIGNO` (+1 o −1) por articulación. La conversión es `servo = offset + signo · escala · q`.
3. Ajusta `LIMITE_SERVO_MIN/MAX` sin forzar los topes mecánicos.
4. Mide la distancia del sensor sin cubo en cada posición y fija `UMBRAL_CM` por debajo de esa lectura, a la mitad de la altura del cubo.
5. Antes de cada rutina, la pila debe estar vacía.

## Uso

### Monitor Serie o terminal TCP

Las órdenes terminan en salto de línea:

| Orden | Acción |
| --- | --- |
| `HOME` | Va a la pose de inicio |
| `AUTO` | Ejecuta la rutina de apilado |
| `STOP` | Detiene el movimiento (siempre disponible) |
| `S,<canal>,<ángulo>` | Mueve un servo (canal 0 a 3) |

Mientras el brazo se mueve, solo acepta `STOP`; las demás órdenes reciben `ERR,OCUPADO`.

Por WiFi, desde un computador en la misma red:

```bash
nc <ip-del-robot> 8080
```

Si la ESP32 no logra conectarse a tu red, crea la suya: `BrazoRobot` (clave `12345678`), con IP `192.168.4.1`.

### Mensajes del robot

| Mensaje | Contenido |
| --- | --- |
| `STAT,<estado>,<base>,<hombro>,<codo>,<pinza>` | Estado y ángulos de los servos |
| `DH,x1,y1,z1,...,x4,y4,z4` | Posición (mm) de hombro, codo, muñeca y garra |
| `RUT,<posición>,<cubos>,<distancia_cm>` | Progreso de la rutina |
| `OK,...` / `ERR,<motivo>` | Respuesta a cada orden |

### App Android

1. Instala la app en el celular (depuración USB o APK).
2. Conecta el celular a la misma red que el robot (o a `BrazoRobot`), escribe la IP, puerto `8080`, y pulsa **Conectar**.

## Cinemática (resumen)

Parámetros D-H estándar (medidas en `config.h`):

| i | θ | d (mm) | a (mm) | α |
| --- | --- | --- | --- | --- |
| 1 | q1 | 128.8 | 0 | 90° |
| 2 | q2 | 0 | 71.8 | 0° |
| 3 | q3 | 0 | 51.5 | 0° |
| 4 (fija) | −φ | 0 | √(dx² + dh²) | 0° |

con dx = 87.0 mm y dh = 13.0 mm para el desfase de la garra. La cinemática inversa es analítica (codo arriba) y rechaza los puntos fuera de alcance o fuera de los límites de servo. Con tres articulaciones, la inclinación de la garra queda determinada por la posición; una garra vertical requeriría una cuarta articulación de muñeca.

## Limitaciones conocidas

- La garra no puede mantenerse vertical (solo 3 grados de libertad).
- El sensor HC-SR04 da lecturas erráticas si queda inclinado; el código usa la mediana de varias lecturas.
- La altura de la pila está limitada por el mínimo del codo, lo que fija qué tan cerca de la base puede ir.
- Una sola app WiFi conectada a la vez.

## Diseño mecanico
Se diseño la pinza para el brazo para permitir la inclusion del sensor

<img width="472" height="403" alt="D76D2A1E-33D2-4BCE-BCEF-BE08673CD9E4_1_201_a" src="https://github.com/user-attachments/assets/1220eb51-0c51-4ccd-b2f4-06d5054328f8" />
<img width="533" height="348" alt="E990E16A-A8D2-404D-9B53-50C7461D7C1D_1_201_a" src="https://github.com/user-attachments/assets/b7618bd6-d7cd-4e65-85c4-75cfef6bb591" />

Fue diseñada en Onshape, un programa CAD online gratuito, Durante la fase de diseño se realizaron varios ajustes según como observamos su funcionamiento y deficiencias hasta llegar al diseño mostrado en las imágenes previas.

Se pueden encontrar los archivos STL para la pinza en el siguiente zip.
[Garra.zip](https://github.com/user-attachments/files/34492960/Garra.zip)


## Licencia

