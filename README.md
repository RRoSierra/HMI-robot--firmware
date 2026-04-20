
# Robot Firmware HMI — Sysmic Robotics SSL

Firmware para un robot de la categoría **Small Size League (SSL)** de RoboCup, desarrollado por **Sysmic Robotics** (UTFSM). Ejecutado sobre un microcontrolador **STM32F767BI** con **FreeRTOS (CMSIS-RTOS v1)**. Controla 4 motores BLDC omnidireccionales mediante cinemática inversa, recibe comandos por radio nRF24L01+ desde un computador central, detecta posesión de pelota con sensor láser ToF, y ejecuta disparos de kicker electromecánico.

---

## Información del Proyecto

| Campo | Valor |
|---|---|
| MCU | STM32F767BITX (Cortex-M7, 216 MHz, FPU, 512 KB RAM, 2 MB Flash) |
| Reloj (HSE) | 8 MHz cristal externo → PLL → 216 MHz SYSCLK |
| APB1 | 54 MHz (SYSCLK/4) |
| APB2 | 108 MHz (SYSCLK/2) |
| RTOS | FreeRTOS v10+ (CMSIS-RTOS v1 API) |
| HAL | STM32F7xx HAL Driver |
| Toolchain | STM32CubeIDE / arm-none-eabi-gcc |
| Archivo .ioc | `robot-firmware.ioc` |
| Linker Script | `STM32F767BITX_FLASH.ld` |

---

## Arquitectura de Software

### Tareas FreeRTOS

El sistema se estructura en **4 tareas RTOS** con prioridades diferenciadas:

| Tarea | Función | Prioridad | Stack (words) | Período |
|---|---|---|---|---|
| `driveTask` | `DriveFunction()` | AboveNormal | 128 | 1 ms (`PID_SAMPLE_TIME`) |
| `radioTask` | `RadioFunction()` | Normal | 128 | Event-driven (cola `nrf24CheckHandle`) |
| `kickTask` | `KickFunction()` | Low | 128 | Event-driven (cola `kickQueueHandle`) |
| `ballDetectorTask` | `BallDetectorFunction()` | Low | 128 | ~1 ms (`osDelay(1)`) |

### Sincronización RTOS

| Recurso | Tipo | Propósito |
|---|---|---|
| `nrf24CheckHandle` | Message Queue (16 elem.) | Señal de `driveTask` → `radioTask` para procesar radio |
| `kickQueueHandle` | Message Queue (1 elem.) | Señal de `driveTask` → `kickTask` para disparar kicker |
| `kickFlagHandle` | Mutex | Protege `kick_flag` (estado de carga del capacitor) |

---

## Modelo Cinemático

### Configuración Omnidireccional (4 ruedas)

El robot usa una base omnidireccional con **4 ruedas** dispuestas en ángulos fijos respecto al frente del robot:

| Rueda | Ángulo | Constante |
|---|---|---|
| Motor 1 | 55° | `WHEEL_ANGlE_1` |
| Motor 2 | 125° | `WHEEL_ANGlE_2` |
| Motor 3 | 235° | `WHEEL_ANGlE_3` |
| Motor 4 | 305° | `WHEEL_ANGlE_4` |

### Cinemática Inversa

La **matriz cinemática** `kinematic[4][3]` se calcula en `main()` al inicio y transforma velocidades del cuerpo `[Vx, Vy, Vω]` en velocidades de cada rueda:

```
kinematic[i][0] = -sin(θᵢ)      ← componente Vx
kinematic[i][1] =  cos(θᵢ)      ← componente Vy
kinematic[i][2] =  ROBOT_RADIO   ← componente Vω (radio del robot)
```

Para cada rueda `i`:

```
v_rueda_i = -sin(θᵢ) · Vx + cos(θᵢ) · Vy + R · ω
```

Donde:
- `Vx`, `Vy`: Velocidad lineal en el plano del robot [m/s]
- `ω`: Velocidad angular [rad/s]
- `R = ROBOT_RADIO = 0.08215 m` (radio del robot)

### Constantes del Robot

| Parámetro | Valor | Unidad |
|---|---|---|
| `ROBOT_RADIO` | 0.08215 | m |
| `ROBOT_MAX_LINEAR_VEL` | 2.5 | m/s |
| `ROBOT_MAX_LINEAR_ACC` | 1.5 | m/s (por ciclo de control) |
| `WHEEL_RADIO` | 0.02704 | m |
| `WHEEL_MAX_SPEED_RPS` | 15.0 | rev/s |
| `WHEEL_MAX_SPEED_RAD` | 2π × 15.0 ≈ 94.25 | rad/s |
| `MOTOR_NOMINAL_SPEED` | 2π × (5240/60) ≈ 548.6 | rad/s |
| `ENCODER_CPR` | 4.0 × 2048 = 8192 | cuentas/rev |
| `PID_SAMPLE_TIME` | 1.0 | ms |

---

## Flujo de Datos — Movimiento del Robot

### Diagrama de Flujo Completo

```
┌──────────────┐    nRF24L01+     ┌──────────────┐
│  PC Central  │ ─── 2.4 GHz ───►│  radioTask   │
│  (grSim/AI)  │    Canal 0x6B    │              │
└──────────────┘                  └──────┬───────┘
                                         │
                              setSpeed() │ Decodifica paquete:
                              (Vx,Vy,Vω) │  - velocidades
                                         │  - dribbler
                                         │  - kicker
                                         ▼
                                  ┌──────────────┐
                                  │  speed[0..3] │  Velocidades por rueda [m/s]
                                  │  dribbler_sel│
                                  │  kick_sel    │
                                  └──────┬───────┘
                                         │
                              Variables   │  nrf24CheckHandle (queue)
                              globales    │
                                         ▼
                                  ┌──────────────┐
                                  │  driveTask   │ Ejecuta cada 1 ms
                                  │              │
                                  │ Motor_CLDrive│ Para cada rueda:
                                  │  ┌─────────┐ │  1. Leer encoder (TIMx->CNT)
                                  │  │Encoder  │ │  2. Calcular velocidad medida
                                  │  │→ PID    │ │  3. PID: ref vs medida
                                  │  │→ DAC    │ │  4. Salida DAC (MAX5815)
                                  │  │→ DIR pin│ │  5. Ajustar pin dirección
                                  │  └─────────┘ │
                                  └──────┬───────┘
                                         │
                              I2C1 (DAC) │  GPIO (DIR/EN/BRK)
                                         ▼
                                  ┌──────────────┐
                                  │  4 Motores   │
                                  │    BLDC      │
                                  └──────────────┘
```

### Paso a Paso Detallado

1. **Recepción Radio** (`radioTask`):
   - El nRF24L01+ recibe un paquete de 30 bytes por RF (canal `0x6B`, dirección `"sysrx"`).
   - Se extrae el segmento correspondiente al `robot_id` (5 bytes por robot): `rx_data[5 * robot_id ... 5 * robot_id + 4]`.
   - `setSpeed()` decodifica los 3 componentes de velocidad (Vx, Vy, Vω) desde la trama binaria.
   - `getDribbler_speed()` extrae los 3 bits de velocidad del dribbler (8 niveles: 0–7).
   - `getKickerStatus()` extrae el bit de kick.
   - Se envía señal a `nrf24CheckHandle` para sincronizar con `driveTask`.

2. **Decodificación de Velocidad** (`setSpeed()`):
   - Los bytes del paquete se decodifican en velocidades con signo, resolución de 9 bits + signo, escaladas por `/100.0` → unidades [m/s] y [rad/s].
   - Se aplica **limitación de velocidad máxima** (`ROBOT_MAX_LINEAR_VEL = 2.5 m/s`).
   - Se aplica **limitación de aceleración** (`ROBOT_MAX_LINEAR_ACC = 1.5`).
   - La **cinemática inversa** transforma `[Vx, Vy, Vω]` → `speed[0..3]` en m/s por rueda.

3. **Control de Motores** (`driveTask`):
   - Ejecuta cada 1 ms (`PID_SAMPLE_TIME`).
   - Para cada uno de los 4 motores llama a `Motor_CLDrive()`:
     - **Encoder**: Lee el registro del timer (`TIMx->CNT`), calcula velocidad angular en rad/s.
     - **PID**: Controlador PID con Kp=12.0, Ki=4.5, Kd=0.0 y anti-windup. Salida: -4095 a +4095.
     - **DAC**: Escribe el valor absoluto de la salida PID al DAC MAX5815 vía I2C1.
     - **Dirección**: Ajusta el pin GPIO DIR según el signo de la velocidad.
   - Paralelamente controla el **dribbler** mediante un segundo DAC (MAX5815 en I2C2).

4. **Respuesta al PC**:
   - Después de procesar, `radioTask` conmuta el nRF24 a modo TX.
   - Envía paquete de 32 bytes con: `robot_id`, `ball_posession`, `ball_range`.
   - Retorna a modo RX.

### Formato del Paquete de Comando (RX desde PC)

Paquete de 30 bytes, conteniendo datos para hasta 6 robots. Por cada robot, 5 bytes:

| Byte | Bits | Contenido |
|---|---|---|
| 0 | [1] | Kick select (1=disparar) |
| 0 | [4:2] | Dribbler speed (3 bits → 0–7 niveles) |
| 1 | [6:0] | Vx magnitude (7 bits LSB) |
| 1 | [7] | Vx signo (0=+, 1=-) |
| 2 | [6:0] | Vy magnitude (7 bits LSB) |
| 2 | [7] | Vy signo |
| 3 | [6:0] | Vω magnitude (7 bits LSB) |
| 3 | [7] | Vω signo |
| 4 | [7:6] | Vx bits [8:7] (2 bits MSB) |
| 4 | [5:4] | Vy bits [8:7] (2 bits MSB) |
| 4 | [3:0] | Vω bits [10:7] (4 bits MSB) |

Velocidad final = ±(MSB_bits << 7 | LSB_7bits) / 100.0 → [m/s] o [rad/s]

### Formato del Paquete de Respuesta (TX al PC)

Paquete de 32 bytes:

| Byte | Bits | Contenido |
|---|---|---|
| 0 | [7:5] | `robot_id` (3 bits) |
| 0 | [0] | `ball_posession` (1=tiene pelota) |
| 1–2 | — | `ball_range` (uint16_t, distancia en mm) |
| 3–31 | — | Reservado (cero) |

---

## Mapa de Pines Completo

### Periféricos de Comunicación

#### I2C1 — DAC de Motores Principales (MAX5815)
| Señal | Pin | Puerto |
|---|---|---|
| SCL | PB6 | GPIOB |
| SDA | PB7 | GPIOB |

**Dispositivo**: MAX5815 (DAC 12-bit, 4 canales). Referencia interna 2.0V.
**Función**: Controla la tensión de alimentación de los 4 drivers de motor BLDC (salidas A, B, C, D → motores 1, 2, 3, 4).

#### I2C2 — DAC del Dribbler (MAX5815)
| Señal | Pin | Puerto |
|---|---|---|
| SDA | PF0 | GPIOF |
| SCL | PF1 | GPIOF |

**Dispositivo**: MAX5815 (DAC 12-bit). Referencia interna 2.0V.
**Función**: Controla la velocidad del motor dribbler (solo canal A).

#### I2C3 — Sensor de Rango VL6180X
| Señal | Pin | Puerto |
|---|---|---|
| SCL | PH7 | GPIOH |
| SDA | PH8 | GPIOH |

**Dispositivo**: VL6180X (sensor ToF, rango ~0–200 mm). Dirección I2C: `0x29`.
**Función**: Detecta posesión de pelota midiendo distancia. Umbral: `VL6180X_THRESHOLD = 65 mm`.

#### SPI1 — Radio nRF24L01+
| Señal | Pin | Puerto |
|---|---|---|
| SCK | PA5 | GPIOA |
| MISO | PA6 | GPIOA |
| MOSI | PA7 | GPIOA |
| CSN | PG10 | GPIOG |
| CE | PG9 | GPIOG |

**Configuración SPI**: Master, CPOL=Low, CPHA=1Edge, 4-bit frame, Prescaler /16, MSB first.
**Canal RF**: `0x6B` (107 decimal, ~2.507 GHz).
**Dirección RX**: `"sysrx"` (5 bytes). **Dirección TX**: `"systx"` + robot_id como último byte.
**Payload**: 30 bytes RX, 32 bytes TX. Auto-Ack deshabilitado.

#### UART5 — Comunicación Serie (Debug/HMI)
| Señal | Pin | Puerto |
|---|---|---|
| TX | PB13 | GPIOB |
| RX | PD2 | GPIOD |

**Configuración**: 115200 baud, 8N1, MSB first habilitado.
**DMA**: TX por DMA1_Stream7, Channel 4.

---

### Encoders de Motores (Timer en modo Encoder)

Todos configurados en **Encoder Mode TI12** (cuadratura, ambos flancos), Period=65535, Prescaler=0.

| Motor | Timer | Pin CH1 | Pin CH2 | Alternate Function |
|---|---|---|---|---|
| Motor 1 | TIM3 | PC6 | PC7 | AF2_TIM3 |
| Motor 2 | TIM8 | PI5 | PI6 | AF3_TIM8 |
| Motor 3 | TIM2 | PA0 | PA1 | AF1_TIM2 |
| Motor 4 | TIM5 | PH10 | PH11 | AF2_TIM5 |

**Resolución**: 4 × 2048 = 8192 CPR (cuentas por revolución).

---

### GPIOs de Control de Motores

Cada motor tiene 3 señales de control GPIO (push-pull, output):

| Motor | Enable Pin | Direction Pin | Brake Pin |
|---|---|---|---|
| Motor 1 | PA10 | PA9 | PA8 |
| Motor 2 | PC11 | PC12 | PD0 |
| Motor 3 | PK7 | PG15 | PK6 |
| Motor 4 | PF5 | PF3 | PF4 |

**Lógica**:
- **Enable**: HIGH = motor habilitado, LOW = deshabilitado.
- **Direction**: HIGH = sentido positivo, LOW = sentido negativo.
- **Brake**: HIGH = sin freno (liberado), LOW = freno activo.

---

### GPIOs del Dribbler

| Función | Pin | Puerto | Estado Inicial |
|---|---|---|---|
| Dribbler Enable | PJ8 | GPIOJ | SET (habilitado) |
| Dribbler Dir | PJ7 | GPIOJ | RESET |
| Dribbler Brake | PJ6 | GPIOJ | SET (sin freno) |

**Velocidades**: 7 niveles DAC predefinidos: `{0, 170, 341, 511, 682, 852, 1023}` (rango 0–1023 del DAC).

---

### GPIOs del Kicker

| Función | Pin | Puerto | Descripción |
|---|---|---|---|
| Carga Capacitor | PJ4 | GPIOJ | HIGH=cargando, LOW=detenido |
| Disparo Kicker | PF11 | GPIOF | Pulso de 10 ms para disparar |

**Ciclo de kickeo**:
1. Se activa PJ4 (HIGH) durante 4 segundos para cargar el capacitor.
2. Se desactiva PJ4 (LOW) y se marca `kick_flag = KICKER_CHARGED`.
3. Cuando `driveTask` recibe comando de kick y la carga está lista, envía señal por cola.
4. `kickTask` activa PF11 (HIGH) durante 10 ms y luego lo desactiva.
5. Se marca `kick_flag = KICKER_DISCHARGED` y se repite el ciclo.

---

### LEDs de Estado (Board)

| LED | Pin | Puerto |
|---|---|---|
| LED 1 | PI12 | GPIOI |
| LED 2 | PI13 | GPIOI |
| LED 3 | PI14 | GPIOI |

Se usan para indicar inicio del sistema (parpadeo 5× al boot).

---

### DIP Switch — Selección de Robot ID

El ID del robot (0–63, 6 bits) se lee al inicio de `driveTask` mediante un DIP switch de 6 posiciones:

| Bit | Pin | Puerto |
|---|---|---|
| Bit 0 | PJ1 | GPIOJ |
| Bit 1 | PJ0 | GPIOJ |
| Bit 2 | PI15 | GPIOI |
| Bit 3 | PB2 | GPIOB |
| Bit 4 | PC4 | GPIOC |
| Bit 5 | PH4 | GPIOH |

Todos configurados como **GPIO Input, No Pull**.

---

## Resumen de Periféricos Utilizados

### Buses de Comunicación Activos

| Bus | Instancia | Pines | Uso | Dispositivo |
|---|---|---|---|---|
| I2C | I2C1 | PB6/PB7 | DAC motores principales | MAX5815 (4ch, 12-bit) |
| I2C | I2C2 | PF0/PF1 | DAC motor dribbler | MAX5815 (1ch) |
| I2C | I2C3 | PH7/PH8 | Sensor de rango | VL6180X (ToF) |
| SPI | SPI1 | PA5/PA6/PA7 | Radio 2.4 GHz | nRF24L01+ |
| UART | UART5 | PB13/PD2 | Debug / HMI | USB-UART externo |
| DMA | DMA1_Stream7 | — | TX de UART5 | — |

### Timers Activos

| Timer | Modo | Uso | Pines |
|---|---|---|---|
| TIM1 | HAL Tick (timebase) | Base de tiempo de HAL (SysTick delegado a RTOS) | — |
| TIM2 | Encoder TI12 | Encoder Motor 3 | PA0/PA1 |
| TIM3 | Encoder TI12 | Encoder Motor 1 | PC6/PC7 |
| TIM5 | Encoder TI12 | Encoder Motor 4 | PH10/PH11 |
| TIM8 | Encoder TI12 | Encoder Motor 2 | PI5/PI6 |

### Puertos GPIO Utilizados (resumen por puerto)

| Puerto | Pines Utilizados | Función |
|---|---|---|
| GPIOA | PA0, PA1, PA5, PA6, PA7, PA8, PA9, PA10 | Encoder 3, SPI1, Motor 1 (brake/dir/enable) |
| GPIOB | PB2, PB6, PB7, PB12, PB13 | DIP SW bit3, I2C1, reservado, UART5 TX |
| GPIOC | PC4, PC6, PC7, PC11, PC12 | DIP SW bit4, Encoder 1, Motor 2 (enable/dir) |
| GPIOD | PD0, PD2 | Motor 2 brake, UART5 RX |
| GPIOF | PF0, PF1, PF3, PF4, PF5, PF11 | I2C2, Motor 4 (dir/brake/enable), Kicker disparo |
| GPIOG | PG9, PG10, PG15 | nRF24 CE, nRF24 CSN, Motor 3 dir |
| GPIOH | PH4, PH7, PH8, PH10, PH11 | DIP SW bit5, I2C3, Encoder 4 |
| GPIOI | PI5, PI6, PI12, PI13, PI14, PI15 | Encoder 2, LED 1/2/3, DIP SW bit2 |
| GPIOJ | PJ0, PJ1, PJ4, PJ6, PJ7, PJ8 | DIP SW bit0/1, Kicker carga, Dribbler (brake/dir/enable) |
| GPIOK | PK6, PK7 | Motor 3 (brake/enable) |

---

## Controlador PID

Cada motor tiene un controlador PID independiente ejecutado a 1 kHz:

| Parámetro | Valor |
|---|---|
| Kp | 12.0 |
| Ki | 4.5 |
| Kd | 0.0 |
| Output Max | +4095 (resolución del DAC 12-bit) |
| Output Min | -4095 |
| Integral Max | outputMax / 5.0 = 819.0 |
| Sample Time | 0.001 s (1 ms) |
| Dead Zone | ±4.0 (valores menores se fuerzan a 0) |

**Referencia**: velocidad en rad/s (`speed[m/s] × SPEED_CNT_RATIO`, donde `SPEED_CNT_RATIO = 1/WHEEL_RADIO`).
**Medida**: velocidad angular del encoder en rad/s.
**Salida**: valor DAC 0–4095 que controla voltaje al driver del motor.

---

## Detección de Pelota (Ball Detector)

- **Sensor**: VL6180X (Time-of-Flight), conectado por I2C3.
- **Muestreo**: ~1 kHz (`osDelay(1)`).
- **Filtro**: Media móvil de 10 muestras.
- **Umbral**: Si `ball_range < 65 mm` → `ball_posession = 0x01`.
- **Uso**: Se reporta al PC central en el paquete TX de radio.

---

## Estructura del Código Fuente

```
Core/
├── Inc/
│   ├── main.h                  # Includes globales, prototipos
│   ├── system_globals.h        # Todas las variables globales, externs de periféricos
│   ├── FreeRTOSConfig.h        # Configuración del RTOS
│   ├── drive_task.h            # Prototipo DriveFunction(), setSpeed()
│   ├── radio_task.h            # Prototipo RadioFunction(), updateBuffer()
│   ├── kick_task.h             # Prototipo KickFunction(), getDribbler_speed(), getKickerStatus()
│   ├── ball_detector_task.h    # Prototipo BallDetectorFunction()
│   └── stm32f7xx_*.h           # HAL config, interrupt handlers
├── Src/
│   ├── main.c                  # Entry point, init periféricos, init cinemática, crear tareas RTOS
│   ├── system_init.c           # Implementación de MX_*_Init() (clocks, GPIO, I2C, SPI, TIM, UART)
│   ├── system_globals.c        # Definición de todas las variables globales
│   ├── drive_task.c            # Tarea de control: PID, DAC, encoders, cinemática
│   ├── radio_task.c            # Tarea de radio: nRF24 RX/TX, parseo de paquetes
│   ├── kick_task.c             # Tarea de kicker: carga capacitor, disparo
│   ├── ball_detector_task.c    # Tarea de detección de pelota: VL6180X, media móvil
│   ├── stm32f7xx_hal_msp.c    # MSP Init: asignación de pines a periféricos (GPIO AF)
│   └── stm32f7xx_it.c         # Interrupt handlers
│
Tools/                          # Bibliotecas de hardware (drivers reutilizables)
├── motor.c / motor.h           # Abstracción de motor: init, enable, brake, OL/CL drive, set voltage
├── encoder.c / encoder.h       # Lectura de encoder incremental, cálculo de velocidad angular
├── PID.c / PID.h               # Controlador PID genérico con anti-windup
├── MAX581x.c / MAX581x.h       # Driver I2C para DAC MAX5814/MAX5815 (12-bit, 4 canales)
├── nrf24.c / nrf24.h           # Driver SPI para transceiver nRF24L01+
├── vl6180x.c / vl6180x.h      # Driver I2C para sensor ToF VL6180X
└── board.c / board.h           # Funciones de placa: LEDs, DIP switch (robot ID)
```

---

## Dependencias

- STM32CubeIDE (o entorno compatible con STM32F7)
- FreeRTOS (integrado vía STM32CubeMX)
- HAL STM32F7xx Drivers
- Módulo nRF24L01+ (SPI, 2.4 GHz)

---

## Compilación y Uso

1. Abrir el proyecto en STM32CubeIDE.
2. Compilar con el perfil Debug o Release.
3. Cargar el firmware al microcontrolador vía ST-Link.
4. Conectar sensores, motores, DACs y módulo de radio según el pinout documentado arriba.

---

## Notas para Modificación

### Para cambiar los ángulos de las ruedas
Editar los defines en `Tools/motor.h`:
```c
#define WHEEL_ANGlE_1  55.0f * M_PI / 180.0f
#define WHEEL_ANGlE_2  125.0f * M_PI / 180.0f
#define WHEEL_ANGlE_3  235.0f * M_PI / 180.0f
#define WHEEL_ANGlE_4  305.0f * M_PI / 180.0f
```
La matriz cinemática se recalcula automáticamente en `main()`.

### Para ajustar el PID
Los parámetros se configuran en `Core/Src/drive_task.c` dentro de `DriveFunction()`:
```c
pidParams.Kp = 12.0f;
pidParams.Ki = 4.5f;
pidParams.Kd = 0.0f;
```

### Para cambiar el canal de radio
Modificar en `Core/Inc/system_globals.h`:
```c
#define nRF24L01_SYSMIC_CHANNEL 0x6B
```

### Para cambiar el umbral de detección de pelota
Modificar en `Core/Inc/system_globals.h`:
```c
#define VL6180X_THRESHOLD 65  // en mm
```

### Para agregar un nuevo periférico I2C/SPI/UART
1. Declarar el handle en `Core/Inc/system_globals.h` (extern) y `Core/Src/system_globals.c` (definición).
2. Crear la función `MX_XXX_Init()` en `Core/Src/system_init.c`.
3. Agregar la configuración MSP (pines GPIO) en `Core/Src/stm32f7xx_hal_msp.c`.
4. Llamar a `MX_XXX_Init()` desde `main()`.

### Para agregar una nueva tarea RTOS
1. Crear archivos `nueva_task.h` y `nueva_task.c` en `Core/Inc/` y `Core/Src/`.
2. Declarar el handle en `Core/Inc/system_globals.h`.
3. Crear la tarea en `main()` con `osThreadDef()` / `osThreadCreate()`.
4. Agregar los archivos al Makefile / proyecto STM32CubeIDE.

---

## Licencia

Proyecto académico de **Sysmic Robotics**. Uso restringido a fines educativos e investigación.