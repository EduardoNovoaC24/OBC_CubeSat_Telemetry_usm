Estoy depurando un driver SX1262 (STM32L476 + FreeRTOS) para un módulo Waveshare Pico-LoRa-SX1262-868M. Ya descarté varias hipótesis de software y hardware básico; necesito que pienses específicamente en hipótesis de SOFTWARE/SECUENCIA DE COMANDOS que aún no haya cubierto (el lado de alimentación/hardware lo estoy verificando en paralelo con osciloscopio, así que no es el foco de esta consulta).

## Hardware
- MCU: STM32L476 (Nucleo-L476RG), FreeRTOS.
- Módulo: Waveshare Pico-LoRa-SX1262-868M, alimentado a 5V, GND común.
- Pinout: NSS=PA4 (GPIO manual), SCK=PA5, MISO=PA6, MOSI=PA7 (SPI1), BUSY=PB0 (GPIO input), DIO1=PB1 (EXTI), NRESET=PC0 (GPIO output).
- SPI1: modo 0 (CPOL=0, CPHA=1EDGE), NSS software, prescaler 64 (~1.25MHz).

## Síntoma (100% reproducible, no intermitente)
El chip llega a STBY_XOSC confirmado y estable (status=0x32, mode=3) justo antes de transmitir. Al ejecutar el comando SET_TX, inmediatamente después:
- status = 0x80 (modo no decodificable como STBY/TX normal)
- OpError = 0x0020 → bit XOSC_START_ERR=1, todos los demás bits de error en 0
- Nunca se llega a IRQ TX_DONE
- El firmware detecta esto y hace reset+reinit completo del chip como recuperación

Log representativo:
```
[SX] PrepareForTx: BUSY=0 status=0x32 mode=3 OpError=0x0000 SW=0x3444 CLAMP=0xFE OCP=0x38
[SX] tras SET_STANDBY(XOSC): status=0x32 mode=3 (STBY_XOSC OK)
[SX] justo antes de SET_TX: CLAMP=0xFE OCP=0x38
[SX] syncword justo antes de SET_TX: 0x3444
[SX] SET_TX timeout_hw=5000ms units=0x04E200
[SX] status DESPUES de SET_TX = 0x80
[SX] OpError=0x0020 RC64K=0 RC13M=0 PLL_CAL=0 ADC_CAL=0 IMG_CAL=0 XOSC_START=1 PLL_LOCK=0 PA_RAMP=0
```

## Configuración de radio
- Frecuencia: 916.8 MHz, packet type LoRa, syncword público LoRaWAN 0x3444.
- DIO2 como RF switch (SetDio2AsRfSwitchCtrl=0x01), una sola vez en Init.
- RegulatorMode = LDO (con DC-DC el radio se quedaba volviendo a STBY_RC antes del TX en este montaje — se eligió LDO empíricamente).
- CalibrateImage 902-928 MHz.
- TCXO controlado por DIO3, voltaje 0x07 (1.8V), timeout de HW 0x001900 (~100ms, unidades de 15.625us).
- PA config: paDutyCycle=0x00, hpMax=0x00 (mínimo absoluto, puesto así para una prueba — valor "normal" previo era 0x02/0x02, equivalente a +14dBm, la config más baja documentada para SX1262).
- SetTxParams: power=2dBm, rampTime=0x04 (200us).
- TX timeout HW: 5000ms.

## Secuencia completa antes de SET_TX (PrepareForTx → SendPacket)
1. SetStandby(XOSC), delay 500ms, verificar mode==STBY_XOSC (lo hace, status=0x32).
2. GetDeviceErrors — limpio en este punto.
3. ApplyTxStaticConfig: SetRfFrequency, SetModulationParams, SetTxParams, SetDioIrqParams.
4. SetPacketParams.
5. ClearIrqStatus.
6. Reescribir manualmente syncword (reg 0x0740/0x0741), TX_CLAMP_CFG (reg 0x08D8=0xFE), OCP (reg 0x08E7=0x38) — esto se hace porque se detectó empíricamente que estos registros "se pierden" (vuelven a su valor default) entre Init() y este punto, sin explicación encontrada aún.
7. Verificación readback de todo lo anterior.
8. (En SendPacket) SetPacketParams otra vez (duplicado), ClearIrqStatus otra vez (duplicado).
9. WriteBuffer con el payload.
10. GetStatus/GetDeviceErrors diagnóstico (limpio aquí también).
11. SetDioIrqParams otra vez.
12. SetStandby(XOSC) OTRA VEZ (segunda transición redundante, ya se hizo en PrepareForTx), delay 500ms, re-verificar mode==STBY_XOSC.
13. ApplyTxStaticConfig OTRA VEZ (tercera vez contando Init).
14. Re-escribir TX_CLAMP_CFG/OCP otra vez, re-verificar.
15. Re-escribir syncword otra vez, re-verificar.
16. SetTx con timeout HW de 5000ms.
17. Falla aquí con XOSC_START_ERR.

## Hipótesis YA DESCARTADAS (con evidencia)
1. **Timeout de SetDio3AsTcxoCtrl muy corto**: se amplió de ~3.9ms a ~100ms. Sin efecto, error idéntico.
2. **Transitorio de corriente de la PA**: se bajó PA a mínimo absoluto (paDutyCycle=0x00, hpMax=0x00). Sin efecto, error idéntico.
3. **El módulo no tiene TCXO real / DIO3 no controla nada físico**: se deshabilitó por completo SetDio3AsTcxoCtrl (dejando que el chip use modo de cristal por defecto). Resultado: el chip NI SIQUIERA llega a STBY_XOSC (se queda en STBY_RC con OpError=0x0020 desde el propio Init). Esto confirma que el TCXO SÍ existe y SÍ es necesario controlarlo por DIO3 — hipótesis descartada, revertido.
4. **Reaplicar SetDio3AsTcxoCtrl justo antes de SET_TX** (igual que se hace con syncword/CLAMP/OCP, por la misma lógica de "estos registros se pierden"): esto produjo un fallo PEOR — BUSY se queda atascado en alto >1s repetidamente (17+ veces) y lecturas SPI posteriores devuelven basura (0xB2 = sin respuesta). Conclusión: SetDio3AsTcxoCtrl no es un registro de memoria pasivo, es un comando que dispara una máquina de estados de arranque del TCXO; reemitirlo mientras el chip ya está en STBY_XOSC (TCXO corriendo) lo deja en un estado inconsistente. Revertido — no reemitir fuera de STBY_RC.
5. **Stress test de SPI/registros**: 100/100 iteraciones OK escribiendo/leyendo 0x0740, 0x0741, 0x08D8, 0x08E7. El bus SPI, NSS y timing son correctos en condiciones normales (sin TX).

## Lo que NO se ha explicado aún (pistas sin resolver)
- ¿Por qué los registros 0x0740/0x0741 (syncword), 0x08D8 (TX_CLAMP_CFG) y 0x08E7 (OCP) "pierden" su valor entre Init() y el momento de PrepareForTx/SendPacket, requiriendo reescritura manual antes de cada TX? Esto se descubrió empíricamente y se trabajó alrededor del síntoma, pero nunca se explicó la causa. Es sospechoso que coincida con la zona donde luego aparece XOSC_START_ERR.
- La doble/triple repetición de SetStandby(XOSC) y ApplyTxStaticConfig entre PrepareForTx y SendPacket (líneas 7-15 arriba) es redundante — ¿podría esta redundancia estar enmascarando o incluso causando el problema en vez de prevenirlo? Nunca se probó simplificar la secuencia a una sola pasada limpia.
- Calibrate(0x7F) se ejecuta en STBY_RC durante el barrido TCXO, ANTES de la transición final a STBY_XOSC — orden estándar del datasheet, pero no se ha verificado si la calibración de PLL para la frecuencia específica de 916.8MHz (post CalibrateImage) queda afectada por algo en el camino hasta SET_TX.
- No se ha instrumentado un log paso a paso DENTRO de la secuencia de PrepareForTx para ver en qué punto exacto (de los 17 pasos arriba) los registros syncword/CLAMP/OCP cambian de valor — solo se sabe que al final de la secuencia ya están corruptos/default, no se sabe cuál paso intermedio los corrompe.

## Lo que pido
Quiero hipótesis de software/firmware (secuencia de comandos SX126x, configuración de IRQ, manejo de mutex/FreeRTOS, posible condición de carrera, uso incorrecto de algún comando, o explicación de por qué los registros "se pierden") que no haya considerado. Sé concreto: qué cambiar exactamente en la secuencia, y qué prueba mínima haría para confirmar o descartar cada hipótesis nueva, dado que ya descarté las 5 hipótesis de arriba con evidencia empírica.
