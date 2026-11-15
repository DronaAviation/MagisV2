# Graph Report - MagisV2  (2026-11-15)

## Corpus Check
- 431 files · ~832,773 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 5121 nodes · 14025 edges · 244 communities (228 shown, 16 thin omitted)
- Extraction: 85% EXTRACTED · 15% INFERRED · 0% AMBIGUOUS · INFERRED: 2144 edges (avg confidence: 0.85)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `c344387b`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- io/serial: serialPortFunction_e
- drivers/sc18is602b: SC18IS601B_SPI_Speed
- USB-FS/core: DEVICE_INFO
- drivers/sound_beeper: beeperConfig_s
- vcp/prop: RESULT
- drivers/pwm_mapping: pwmOutputConfiguration_s
- API/X1Ranging: X1Ranging_P
- API/Motor: Rev_Motor_Gpio
- drivers/sc18is602b: SC18IS602B
- drivers/pwm_mapping: sonarGPIOConfig_s
- drivers/timer: timerCCHandlerRec_s
- flight/mixer: mixerMode
- io/serial_msp: mspPort_s
- flight/motor: Rev_Motor_Gpio
- flight/altitudehold: AltTofL1x
- API/XRanging: laser_sensors
- CMSIS/stm32f30x: IRQn
- io/rc_controls: adjustmentConfig_u
- drivers/timer: timerOvrHandlerRec_s
- io/rc_controls: adjustmentSelectConfig_s
- io/rc_controls: adjustmentRange_s
- io/rc_controls: adjustmentStepConfig_s
- rx/rx: rxConfig_s
- drivers/barometer: baro_s
- blackbox/blackbox_fielddefs: flightLogEventData_u
- flight/failsafe: failsafeState_s
- flight/pid: pidProfile_s
- drivers/pwm_mapping: drv_pwm_config_s
- blackbox/blackbox: blackboxSlowState_s
- blackbox/blackbox: blackboxMainState_s
- drivers + API: system, runtime_config
- io/rc_controls: rc_alias
- drivers/accgyro: gyro_s
- blackbox/blackbox_io: BlackboxDevice
- blackbox/blackbox: BlackboxState
- blackbox/blackbox_fielddefs: FlightLogFieldPredictor
- blackbox/blackbox: userBlackboxMainState_s
- VL53L1X/preset_setup: VL53L1_Tuning_t
- io/gps: gpsData_s
- rx/rx: rxChannelRangeConfiguration_s
- blackbox/blackbox_fielddefs: FlightLogEvent
- API/XRanging: XRanging_P
- blackbox/blackbox_fielddefs: blackboxDeltaFieldDefinition_s
- flight/mixer: servoParam_s
- io/rc_controls: controlRateConfig_s
- API/ObjectAvoidanceCore: oaCoreState
- blackbox/blackbox_fielddefs: FlightLogFieldEncoding
- flight/mixer: servoMixer_s
- flight/navigation: gpsProfile_s
- sensors/battery: batteryConfig_s
- API/Scheduler-Timer: Interval
- blackbox/blackbox_fielddefs: blackboxConditionalFieldDefinit…
- flight/imu: imuRuntimeConfig_s
- blackbox/blackbox_fielddefs: blackboxSimpleFieldDefinition_s
- common/maths: stdev_s
- flight/failsafe: failsafeConfig_s
- blackbox/blackbox_fielddefs: flightLogEvent_inflightAdjustme…
- drivers/adc: adc_config_s
- drivers/adc: drv_adc_config_s
- flight/mixer: flight3DConfig_s
- flight/mixer: mixer_s
- flight/mixer: motorMixer_s
- io/rc_controls: adjustmentConfig_s
- io/rc_controls: rcControlsConfig_s
- sensors/barometer: barometerConfig_s
- sensors/sensors: sensorAlignmentConfig_s
- blackbox/blackbox: blackboxFieldDefinition_s
- blackbox/blackbox: blackboxGpsState_s
- blackbox/blackbox_fielddefs: flightLogEvent_gtuneCycleResult…
- common/color: hsvColor_s
- common/color: rgbColor24bpp_s
- common/maths: fp_angles
- common/maths: fp_vector
- flight/mixer: mixerRules_s
- flight/mixer: mixerConfig_s
- io/beeper: beeperTableEntry_s
- io/ledstrip: ledConfig_s
- flight + io: rc_controls, blackbox
- io/serial_msp: box_e
- sensors/sensors: int16_flightDynamicsTrims_s
- blackbox/blackbox_fielddefs: flightLogEvent_loggingResume_s
- blackbox/blackbox_fielddefs: FlightLogFieldSign
- flight/imu: accDeadband_s
- flight/imu: rollAndPitchInclination_s
- io/rc_controls: channelRange_s
- sensors/acceleration: rollAndPitchTrims_s
- API + drivers: Localisation, Motor
- flight/mixer: airplaneConfig_s
- io/gimbal: gimbalConfig_s
- sensors/gyro: gyroConfig_s
- USB-FS/core: CONTROL_STATE
- drivers + StdPeriph: dma, serial
- io/gps: gpsInitUblox()
- drivers/serial: serialPort_s
- flight/lowpass: lowpass_s
- io/gps: gpsConfig_s
- API + command: Scheduler-Timer, command
- drivers/accgyro_mpu: lpf_e
- io/gps: gpsCoordinateDDDMMmmmm_s
- API/Motor: bidirectional_motor
- drivers/tof_dev_area: tofDevArea_u
- drivers/pwm_rx: ppmDevice_s
- API/XRanging: laser_e
- rx/rx: crsfFrameStatus()
- flight + io: mixer, mw
- drivers/pwm_output: gpioInit()
- sensors/battery: annexCode()
- rx/rx: detectAndApplySignalLossBehavio…
- io/flashfs: flashfsWriteBuffers()
- CMSIS/arm_math: __INLINE
- StdPeriph/flash: writeEEPROM()
- API/FC-Data-Sensors: Sensor_Get()
- telemetry/frsky: handleFrSkyTelemetry()
- drivers/serial_softserial: serialPort_t
- config/config: featureSet()
- telemetry/hott: handleHoTTTelemetry()
- flight: imu, filter
- VL53L1X/api_preset_modes: VL53L1_set_preset_mode()
- drivers/bus_i2c_stm32f30x: i2cRead_v2()
- USB-FS: regs, mem
- common + flight: maths, imu
- VL53L0X/api: VL53L0X_DEV
- VL53L0X/api_calibration: VL53L0X_WrByte()
- drivers/paw3903_opticflow: paw3903_read_reg()
- flight/lowpass: lowpassFixed()
- API + drivers: Motor, RxConfig
- sensors/acceleration: flightAccelCalData_T
- CMSIS/cmFunc: __STATIC_INLINE
- CMSIS/cm0: __STATIC_INLINE
- CMSIS/cmInstr: __STATIC_INLINE
- telemetry + io: smartport, serial
- StdPeriph/dac: DAC_TypeDef
- API + common: FC-Control-Status, maths
- flight/failsafe: useFailsafeConfig()
- CMSIS/gpio: GPIO_DeInit()
- CMSIS: sc000, cm0plus
- VL53L0X/api: VL53L0X_Error
- USB-FS/core: NoData_Setup0()
- StdPeriph/exti: configureMPUDataReadyInterruptH…
- StdPeriph/hrtim: HRTIM_ADCTriggerConfig()
- StdPeriph/tim: ws2811LedStripHardwareInit()
- API + drivers: Serial-IO-Uart, opticflow_paw3903
- StdPeriph/opamp: OPAMP_InitTypeDef
- StdPeriph/adc: ADC_TypeDef
- drivers/bridge_sc18is602b: sc18_i2c_write()
- drivers/tof_manager_l1x: tofId_e
- StdPeriph: pwr, wwdg
- drivers: compass_ak09916, opticflow_paw3903
- StdPeriph/crc: CRC_ReverseOutputDataCmd()
- drivers/timer: timerHardware_t
- config + io: config, display
- VL53L1X/register_funcs: VL53L1_init_and_start_range()
- io + user: oled_display, PlutoPilot
- drivers + API: bus_spi, Serial-IO-SPI
- StdPeriph/hrtim: HRTIM_OutputConfig()
- common/printf: tfp_format()
- drivers/bus_i2c_soft: I2C_delay()
- StdPeriph/tim: TIM_ICInit()
- drivers/display_ug2864hsweg01: i2c_OLED_send_cmd()
- CMSIS/arm_math: float32_t
- StdPeriph/hrtim: HRTIM_TimingUnitBase_Config()
- VL53L1X/core: VL53L1_WrByte()
- VL53L1X: api_core, core
- blackbox/blackbox: blackboxWrite()
- io/ledstrip: setLedHsv()
- CMSIS/arm_math: __QADD()
- CMSIS/arm_math: arm_pid_q15()
- VL53L0X/api: VL53L0X_PerformSingleRangingMea…
- API/RGB-LED: RGB_UpdateAnimation()
- flight/altitudehold: Estimate_Get()
- CMSIS/arm_math: q15_t
- StdPeriph/hrtim: HRTIM_ExternalEventConfig()
- flight/mixer: mixerInit()
- StdPeriph/rcc: RCC_APB2PeriphResetCmd()
- io/ledstrip: updateLedStrip()
- sensors/initialisation: sensorsAutodetectmpu()
- io/oled_display: Oled_DrawHLine()
- StdPeriph/rtc: ErrorStatus
- drivers/light_led: void()
- CMSIS/arm_math: arm_bilinear_interp_q15()
- CMSIS/arm_math: arm_bilinear_interp_q31()
- CMSIS/arm_math: arm_bilinear_interp_q7()
- StdPeriph/i2c: FunctionalState
- StdPeriph/usart: USART_TypeDef
- io/serial_cli: cliPrint()
- CMSIS/cm3: __INLINE
- StdPeriph/gpio: GPIO_TypeDef
- USB-FS/regs: Virtual_Com_Port_Reset()
- VL53L1X: ll_def, api_debug
- VL53L1X: api, api_core
- sensors/barometer: apmBaroCalibrate()
- flight/navigation: onGpsNewData()
- io/serial_msp: processInCommand()
- CMSIS/cm4_simd: __STATIC_INLINE
- drivers/tof_manager: xRangingInit()
- config + sensors: config, barometer
- blackbox/blackbox: startBlackbox()
- io/serial_cli: cliDump()
- API/Oled: Oled_DrawLine()
- VL53L1X/register_funcs: VL53L1_ReadMulti()
- API/RxConfig: Receiver_Aux_Config()
- flight: altitudehold, posEstimate
- sensors/acceleration: updateAccelerationReadings()
- flight + API: pid, acceleration
- vcp/hw_config: usbVcpOpen()
- StdPeriph/hrtim: HRTIM_TypeDef
- rx + API: API-Utils, statusindicator
- API/X1Ranging: x1RangingInit()
- telemetry: frsky, hott
- io/serial_cli: cliMotorMix()
- common/atomic: __basepriSetMemRetVal()
- telemetry: serial, frsky
- flight/altitudehold: checkReadingFor()
- rx/xbus: xBusDataReceive()
- blackbox: blackbox_io, blackbox
- drivers/light_ws2811strip: ws2811UpdateStrip()
- drivers/flash_m25p16: m25p16_waitForReady()
- VL53L0X/api: VL53L0X_RdByte()
- CMSIS/rcc: RCC_GetFlagStatus()
- VL53L1X: wait, platform
- flight/posControl: resetPosIntegral()
- io/oled_display: Oled_Is_Mode()
- StdPeriph/can: CAN_TypeDef
- VL53L1X/def: ConvertModeFromLLD()
- StdPeriph/tim: TIM_TypeDef
- StdPeriph/comp: COMP_InitTypeDef
- StdPeriph/spi: SPI_TypeDef
- VL53L0X/api_core: get_sequence_step_timeout()
- API + blackbox: Debugging, blackbox
- API/FC-Config-Setpoints: Failsafe_disable()
- API: XRanging, Peripheral-GPIO
- API/Debugging: debugPrint()
- rx + io: rx, serial
- CMSIS/cm4: __STATIC_INLINE
- io/beeper: cliPlaySound()
- CMSIS/cm3: __STATIC_INLINE
- CMSIS/sc300: __STATIC_INLINE
- vcp: stm32_it, pwr
- VL53L0X: i2c_platform, platform
- io/ledstrip: reevalulateLedConfig()
- io/display: updateDisplay()
- io/serial_1wire: escHardware_t
- StdPeriph/gpio: BitAction

## God Nodes (most connected - your core abstractions)
1. `VL53L1_Error` - 284 edges
2. `VL53L1_DEV` - 202 edges
3. `FunctionalState` - 162 edges
4. `VL53L0X_Error` - 161 edges
5. `VL53L0X_DEV` - 144 edges
6. `TIM_TypeDef` - 121 edges
7. `HRTIM_TypeDef` - 77 edges
8. `millis()` - 74 edges
9. `init()` - 71 edges
10. `ADC_TypeDef` - 61 edges

## Surprising Connections (you probably didn't know these)
- `init()` --calls--> `plutoInit()`  [INFERRED]
  src/main/main.cpp → PlutoPilot.cpp
- `ws2811LedStripHardwareInit()` --calls--> `TIM_CtrlPWMOutputs()`  [INFERRED]
  src/main/drivers/light_ws2811strip_stm32f30x.c → lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_tim.c
- `pwmOutConfig()` --calls--> `TIM_Cmd()`  [INFERRED]
  src/main/drivers/pwm_output.cpp → lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_tim.c
- `writeEEPROM()` --calls--> `FLASH_ClearFlag()`  [INFERRED]
  src/main/config/config.cpp → lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_flash.c
- `writeEEPROM()` --calls--> `FLASH_Lock()`  [INFERRED]
  src/main/config/config.cpp → lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_flash.c

## Import Cycles
- None detected.

## Communities (244 total, 16 thin omitted)

### Community 1 - "io/serial: serialPortFunction_e"
Cohesion: 0.08
Nodes (38): findSerialPortConfigState_s, findSharedSerialPortState_s, serialConfig_s, serialPortConfig_s, serialPortUsage_s, doesConfigurationUsePort(), findNextSerialPortConfig(), findNextSharedSerialPort() (+30 more)

### Community 101 - "drivers/sc18is602b: SC18IS601B_SPI_Speed"
Cohesion: 0.40
Nodes (5): SC18IS601B_SPI_Speed, SC18IS601B_SPICLK_115_kHz, SC18IS601B_SPICLK_1843_kHz, SC18IS601B_SPICLK_461_kHz, SC18IS601B_SPICLK_58_kHz

### Community 105 - "USB-FS/core: DEVICE_INFO"
Cohesion: 0.07
Nodes (28): DEVICE, DEVICE_INFO, DEVICE_PROP, ENDPOINT_INFO, OneDescriptor, USER_STANDARD_REQUESTS, ControlState, Ctrl_Info (+20 more)

### Community 108 - "drivers/sound_beeper: beeperConfig_s"
Cohesion: 0.33
Nodes (6): beeperConfig_s, gpioMode, gpioPeripheral, gpioPin, gpioPort, isInverted

### Community 11 - "vcp/prop: RESULT"
Cohesion: 0.10
Nodes (18): RESULT, Standard_GetDescriptorData(), USB_SIL_Init(), USB_Cable_Config(), Virtual_Com_Port_Data_Setup(), Virtual_Com_Port_Get_Interface_Setting(), Virtual_Com_Port_GetConfigDescriptor(), Virtual_Com_Port_GetDeviceDescriptor() (+10 more)

### Community 110 - "drivers/pwm_mapping: pwmOutputConfiguration_s"
Cohesion: 0.67
Nodes (3): pwmOutputConfiguration_s, motorCount, servoCount

### Community 114 - "API/Motor: Rev_Motor_Gpio"
Cohesion: 0.50
Nodes (4): Rev_Motor_Gpio, gpio, pin, RCC_AHBPeriph

### Community 115 - "drivers/sc18is602b: SC18IS602B"
Cohesion: 0.10
Nodes (18): SC18IS601B_GPIOPinMode, SC18IS601B_SPI_Mode, SC18IS602B, SC18IS601B_GPIO_MODE_INPUT_ONLY, SC18IS601B_GPIO_MODE_OPEN_DRAIN, SC18IS601B_GPIO_MODE_PUSH_PULL, SC18IS601B_GPIO_MODE_QUASI_BIDIRECTIONAL, SC18IS601B_SPIMODE_0 (+10 more)

### Community 119 - "drivers/pwm_mapping: sonarGPIOConfig_s"
Cohesion: 0.50
Nodes (4): sonarGPIOConfig_s, echoPin, gpio, triggerPin

### Community 120 - "drivers/timer: timerCCHandlerRec_s"
Cohesion: 0.67
Nodes (3): timerCCHandlerRec_s, timerCCHandlerCallback, fn

### Community 121 - "flight/mixer: mixerMode"
Cohesion: 0.08
Nodes (26): mixerMode, MIXER_AIRPLANE, MIXER_ATAIL4, MIXER_BICOPTER, MIXER_CUSTOM, MIXER_CUSTOM_AIRPLANE, MIXER_CUSTOM_TRI, MIXER_DUALCOPTER (+18 more)

### Community 122 - "io/serial_msp: mspPort_s"
Cohesion: 0.12
Nodes (14): mspPort_s, mspSetTelemetryPort(), resetMspPort(), setCurrentPort(), configureMSPTelemetryPort(), c_state, checksum, cmdMSP (+6 more)

### Community 123 - "flight/motor: Rev_Motor_Gpio"
Cohesion: 0.50
Nodes (4): Rev_Motor_Gpio, gpio, pin, RCC_AHBPeriph

### Community 124 - "flight/altitudehold: AltTofL1x"
Cohesion: 0.05
Nodes (33): AltTofL0x, AltTofL1x, tofL1xDownClearNew(), tofL1xDownNew(), tofL1xDownOutOfRange(), tofL1xDownRangeMm(), tofDownClearNew(), tofDownFiltMm() (+25 more)

### Community 129 - "API/XRanging: laser_sensors"
Cohesion: 0.29
Nodes (7): laser_sensors, BACK, DOWN, EXTERNAL, FRONT, LEFT, RIGHT

### Community 13 - "CMSIS/stm32f30x: IRQn"
Cohesion: 0.03
Nodes (56): IRQn, DBGMCU_APB1PeriphConfig(), DBGMCU_APB2PeriphConfig(), DBGMCU_Config(), IWDG_GetFlagStatus(), TIM_SelectOCxM_NoDisable(), ADC_Common_TypeDef, CAN_FIFOMailBox_TypeDef (+48 more)

### Community 133 - "io/rc_controls: adjustmentConfig_u"
Cohesion: 0.40
Nodes (5): adjustmentConfig_u, selectConfig, stepConfig, adjustmentSelectConfig_t, adjustmentStepConfig_t

### Community 144 - "drivers/timer: timerOvrHandlerRec_s"
Cohesion: 0.50
Nodes (4): timerOvrHandlerRec_s, timerOvrHandlerCallback, fn, next

### Community 146 - "io/rc_controls: adjustmentRange_s"
Cohesion: 0.08
Nodes (25): adjustmentRange_s, adjustmentState_s, modeActivationCondition_s, configureAdjustment(), isModeActivationConditionPresent(), isRangeActive(), updateActivatedModes(), updateAdjustmentStates() (+17 more)

### Community 161 - "rx/rx: rxConfig_s"
Cohesion: 0.12
Nodes (16): rxConfig_s, channelRanges, failsafe_channel_configurations, maxcheck, midrc, mincheck, rcmap, rcSmoothing (+8 more)

### Community 167 - "drivers/barometer: baro_s"
Cohesion: 0.15
Nodes (16): baro_s, calculate, get_up, get_ut, measurment_start, read, start_up, start_ut (+8 more)

### Community 176 - "blackbox/blackbox_fielddefs: flightLogEventData_u"
Cohesion: 0.16
Nodes (15): flightLogEvent_syncBeep_s, flightLogEventData_u, blackboxConditionalFieldDefinition_t, blackboxDeltaFieldDefinition_t, blackboxSimpleFieldDefinition_t, flightLogEvent_gtuneCycleResult_t, flightLogEvent_inflightAdjustment_t, flightLogEvent_loggingResume_t (+7 more)

### Community 177 - "flight/failsafe: failsafeState_s"
Cohesion: 0.12
Nodes (16): failsafeState_s, failsafePhase(), failsafePhase_e, failsafeRxLinkState_e, active, events, landingShouldBeFinishedAt, monitoring (+8 more)

### Community 178 - "flight/pid: pidProfile_s"
Cohesion: 0.12
Nodes (16): pidProfile_s, A_level, D8, D_f, dterm_cut_hz, gyro_cut_hz, H_level, H_sensitivity (+8 more)

### Community 188 - "drivers/pwm_mapping: drv_pwm_config_s"
Cohesion: 0.13
Nodes (15): drv_pwm_config_s, airplane, idlePulse, motorPwmRate, sonarGPIOConfig, useCurrentMeterADC, useLEDStrip, useOneshot (+7 more)

### Community 189 - "blackbox/blackbox: blackboxSlowState_s"
Cohesion: 0.20
Nodes (10): blackboxSlowState_s, loadSlowState(), writeSlowFrameIfNeeded(), rxAreFlightChannelsValid(), failsafePhase, flightModeFlags, rxFlightChannelsValid, rxSignalReceived (+2 more)

### Community 192 - "blackbox/blackbox: blackboxMainState_s"
Cohesion: 0.14
Nodes (14): blackboxMainState_s, accSmooth, amperageLatest, axisPID_D, axisPID_I, axisPID_P, gyroADC, logVar (+6 more)

### Community 2 - "drivers + API: system, runtime_config"
Cohesion: 0.05
Nodes (28): serialPortVTable, command_add(), icp10111_calculate(), icp10111_read(), INA219_Config(), INA219_Init(), INA219_RegWrite(), cycleCounterInit() (+20 more)

### Community 201 - "io/rc_controls: rc_alias"
Cohesion: 0.15
Nodes (13): rc_alias, AUX1, AUX2, AUX3, AUX4, AUX5, AUX6, AUX7 (+5 more)

### Community 207 - "drivers/accgyro: gyro_s"
Cohesion: 0.17
Nodes (11): acc_s, gyro_s, init, read, revisionCode, init, read, scale (+3 more)

### Community 208 - "blackbox/blackbox_io: BlackboxDevice"
Cohesion: 0.67
Nodes (3): BlackboxDevice, BLACKBOX_DEVICE_END, BLACKBOX_DEVICE_SERIAL

### Community 209 - "blackbox/blackbox: BlackboxState"
Cohesion: 0.17
Nodes (12): BlackboxState, BLACKBOX_STATE_DISABLED, BLACKBOX_STATE_PAUSED, BLACKBOX_STATE_RUNNING, BLACKBOX_STATE_SEND_GPS_G_HEADER, BLACKBOX_STATE_SEND_GPS_H_HEADER, BLACKBOX_STATE_SEND_HEADER, BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER (+4 more)

### Community 210 - "blackbox/blackbox_fielddefs: FlightLogFieldPredictor"
Cohesion: 0.17
Nodes (12): FlightLogFieldPredictor, FLIGHT_LOG_FIELD_PREDICTOR_0, FLIGHT_LOG_FIELD_PREDICTOR_1500, FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2, FLIGHT_LOG_FIELD_PREDICTOR_HOME_COORD, FLIGHT_LOG_FIELD_PREDICTOR_INC, FLIGHT_LOG_FIELD_PREDICTOR_LAST_MAIN_FRAME_TIME, FLIGHT_LOG_FIELD_PREDICTOR_MINTHROTTLE (+4 more)

### Community 214 - "blackbox/blackbox: userBlackboxMainState_s"
Cohesion: 0.67
Nodes (3): userBlackboxMainState_s, logVar, time

### Community 216 - "VL53L1X/preset_setup: VL53L1_Tuning_t"
Cohesion: 0.18
Nodes (10): VL53L1_Tuning_t, VL53L1_TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER, VL53L1_TUNING_MAX_TUNABLE_KEY, VL53L1_TUNING_MIN_AMBIENT_DMAX_VALID, VL53L1_TUNING_PROXY_MIN, VL53L1_TUNING_SIMPLE_OFFSET_CALIBRATION_REPEAT, VL53L1_TUNING_SINGLE_TARGET_XTALK_SAMPLE_NUMBER, VL53L1_TUNING_SINGLE_TARGET_XTALK_TARGET_DISTANCE_MM (+2 more)

### Community 220 - "io/gps: gpsData_s"
Cohesion: 0.18
Nodes (11): gpsData_s, baudrateIndex, errors, lastLastMessage, lastMessage, messageState, state, state_position (+3 more)

### Community 223 - "rx/rx: rxChannelRangeConfiguration_s"
Cohesion: 0.67
Nodes (3): rxChannelRangeConfiguration_s, max, min

### Community 225 - "blackbox/blackbox_fielddefs: FlightLogEvent"
Cohesion: 0.20
Nodes (10): FlightLogEvent, flightLogEvent_s, FLIGHT_LOG_EVENT_GTUNE_RESULT, FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT, FLIGHT_LOG_EVENT_LOG_END, FLIGHT_LOG_EVENT_LOGGING_RESUME, FLIGHT_LOG_EVENT_SYNC_BEEP, data (+2 more)

### Community 227 - "API/XRanging: XRanging_P"
Cohesion: 0.12
Nodes (8): laser_status, XRanging_P, LASER_BOOT_FAILED, LASER_I2C_ERROR, LASER_NOT_ENABLED, LASER_OK, LASER_OUT_OF_RANGE, triggerThreshold

### Community 229 - "blackbox/blackbox_fielddefs: blackboxDeltaFieldDefinition_s"
Cohesion: 0.22
Nodes (9): blackboxDeltaFieldDefinition_s, condition, fieldNameIndex, Iencode, Ipredict, isSigned, name, Pencode (+1 more)

### Community 230 - "flight/mixer: servoParam_s"
Cohesion: 0.22
Nodes (9): servoParam_s, angleAtMax, angleAtMin, forwardFromChannel, max, middle, min, rate (+1 more)

### Community 231 - "io/rc_controls: controlRateConfig_s"
Cohesion: 0.22
Nodes (9): controlRateConfig_s, dynThrPID, rates, rcExpo8, rcRate8, rcYawExpo8, thrExpo8, thrMid8 (+1 more)

### Community 237 - "API/ObjectAvoidanceCore: oaCoreState"
Cohesion: 0.11
Nodes (16): oaCoreState, userRCassert(), oaCoreDistance(), oaCoreMap(), oaCoreRun(), RcCommand_Get(), RcCommand_Set(), RcData_Get() (+8 more)

### Community 238 - "blackbox/blackbox_fielddefs: FlightLogFieldEncoding"
Cohesion: 0.25
Nodes (8): FlightLogFieldEncoding, FLIGHT_LOG_FIELD_ENCODING_NEG_14BIT, FLIGHT_LOG_FIELD_ENCODING_NULL, FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB, FLIGHT_LOG_FIELD_ENCODING_TAG2_3S32, FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16, FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB, FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB

### Community 239 - "flight/mixer: servoMixer_s"
Cohesion: 0.25
Nodes (8): servoMixer_s, box, inputSource, max, min, rate, speed, targetChannel

### Community 240 - "flight/navigation: gpsProfile_s"
Cohesion: 0.25
Nodes (8): gpsProfile_s, ap_mode, gps_lpf, gps_wp_radius, nav_controls_heading, nav_slew_rate, nav_speed_max, nav_speed_min

### Community 241 - "sensors/battery: batteryConfig_s"
Cohesion: 0.25
Nodes (8): batteryConfig_s, BatteryCapacity, currentMeterType, multiwiiCurrentMeterOutput, vBatMaxVoltage, vBatMinVoltage, vBatWarningVoltage, currentSensor_e

### Community 246 - "API/Scheduler-Timer: Interval"
Cohesion: 0.29
Nodes (4): Interval, loopTime, repeat, time

### Community 248 - "blackbox/blackbox_fielddefs: blackboxConditionalFieldDefinit…"
Cohesion: 0.29
Nodes (7): blackboxConditionalFieldDefinition_s, condition, encode, fieldNameIndex, isSigned, name, predict

### Community 249 - "flight/imu: imuRuntimeConfig_s"
Cohesion: 0.29
Nodes (7): imuRuntimeConfig_s, acc_lpf_factor, acc_unarmedcal, gyro_cmpf_factor, gyro_cmpfm_factor, gyro_lpf_factor, small_angle

### Community 256 - "blackbox/blackbox_fielddefs: blackboxSimpleFieldDefinition_s"
Cohesion: 0.33
Nodes (6): blackboxSimpleFieldDefinition_s, encode, fieldNameIndex, isSigned, name, predict

### Community 257 - "common/maths: stdev_s"
Cohesion: 0.33
Nodes (6): stdev_s, m_n, m_newM, m_newS, m_oldM, m_oldS

### Community 259 - "flight/failsafe: failsafeConfig_s"
Cohesion: 0.33
Nodes (6): failsafeConfig_s, failsafe_delay, failsafe_kill_switch, failsafe_off_delay, failsafe_throttle, failsafe_throttle_low_delay

### Community 263 - "blackbox/blackbox_fielddefs: flightLogEvent_inflightAdjustme…"
Cohesion: 0.40
Nodes (5): flightLogEvent_inflightAdjustment_s, adjustmentFunction, floatFlag, newFloatValue, newValue

### Community 264 - "drivers/adc: adc_config_s"
Cohesion: 0.40
Nodes (5): adc_config_s, adcChannel, dmaIndex, enabled, sampleTime

### Community 265 - "drivers/adc: drv_adc_config_s"
Cohesion: 0.40
Nodes (5): drv_adc_config_s, enableCurrentMeter, enableExternal1, enableRSSI, enableVBat

### Community 269 - "flight/mixer: flight3DConfig_s"
Cohesion: 0.40
Nodes (5): flight3DConfig_s, deadband3d_high, deadband3d_low, deadband3d_throttle, neutral3d

### Community 270 - "flight/mixer: mixer_s"
Cohesion: 0.40
Nodes (4): mixer_s, motor, motorCount, useServo

### Community 271 - "flight/mixer: motorMixer_s"
Cohesion: 0.40
Nodes (5): motorMixer_s, pitch, roll, throttle, yaw

### Community 273 - "io/rc_controls: adjustmentConfig_s"
Cohesion: 0.40
Nodes (5): adjustmentConfig_s, adjustmentFunction, data, mode, adjustmentData_t

### Community 275 - "io/rc_controls: rcControlsConfig_s"
Cohesion: 0.40
Nodes (5): rcControlsConfig_s, alt_hold_deadband, alt_hold_fast_change, deadband, yaw_deadband

### Community 276 - "sensors/barometer: barometerConfig_s"
Cohesion: 0.40
Nodes (5): barometerConfig_s, baro_cf_alt, baro_cf_vel, baro_noise_lpf, baro_sample_count

### Community 277 - "sensors/sensors: sensorAlignmentConfig_s"
Cohesion: 0.40
Nodes (5): sensorAlignmentConfig_s, sensor_align_e, acc_align, gyro_align, mag_align

### Community 285 - "blackbox/blackbox: blackboxFieldDefinition_s"
Cohesion: 0.50
Nodes (4): blackboxFieldDefinition_s, arr, fieldNameIndex, name

### Community 286 - "blackbox/blackbox: blackboxGpsState_s"
Cohesion: 0.50
Nodes (4): blackboxGpsState_s, GPS_coord, GPS_home, GPS_numSat

### Community 287 - "blackbox/blackbox_fielddefs: flightLogEvent_gtuneCycleResult…"
Cohesion: 0.50
Nodes (4): flightLogEvent_gtuneCycleResult_s, gtuneAxis, gtuneGyroAVG, gtuneNewP

### Community 288 - "common/color: hsvColor_s"
Cohesion: 0.50
Nodes (4): hsvColor_s, h, s, v

### Community 289 - "common/color: rgbColor24bpp_s"
Cohesion: 0.50
Nodes (4): rgbColor24bpp_s, b, g, r

### Community 290 - "common/maths: fp_angles"
Cohesion: 0.50
Nodes (4): fp_angles, pitch, roll, yaw

### Community 291 - "common/maths: fp_vector"
Cohesion: 0.50
Nodes (4): fp_vector, X, Y, Z

### Community 295 - "flight/mixer: mixerRules_s"
Cohesion: 0.50
Nodes (3): mixerRules_s, rule, servoRuleCount

### Community 296 - "flight/mixer: mixerConfig_s"
Cohesion: 0.50
Nodes (4): mixerConfig_s, pid_at_min_throttle, yaw_jump_prevention_limit, yaw_motor_direction

### Community 298 - "io/beeper: beeperTableEntry_s"
Cohesion: 0.50
Nodes (4): beeperTableEntry_s, mode, priority, sequence

### Community 299 - "io/ledstrip: ledConfig_s"
Cohesion: 0.50
Nodes (4): ledConfig_s, color, flags, xy

### Community 3 - "flight + io: rc_controls, blackbox"
Cohesion: 0.12
Nodes (33): escAndServoConfig_s, gimbalConfig_s, rxConfig_s, useGyroConfig(), blackboxFieldDefinition_t, blackboxGpsState_t, blackboxMainState_t, userBlackboxMainState_t (+25 more)

### Community 300 - "io/serial_msp: box_e"
Cohesion: 0.50
Nodes (4): box_e, boxId, boxName, permanentId

### Community 301 - "sensors/sensors: int16_flightDynamicsTrims_s"
Cohesion: 0.50
Nodes (4): int16_flightDynamicsTrims_s, pitch, roll, yaw

### Community 310 - "blackbox/blackbox_fielddefs: flightLogEvent_loggingResume_s"
Cohesion: 0.67
Nodes (3): flightLogEvent_loggingResume_s, currentTime, logIteration

### Community 311 - "blackbox/blackbox_fielddefs: FlightLogFieldSign"
Cohesion: 0.67
Nodes (3): FlightLogFieldSign, FLIGHT_LOG_FIELD_SIGNED, FLIGHT_LOG_FIELD_UNSIGNED

### Community 313 - "flight/imu: accDeadband_s"
Cohesion: 0.67
Nodes (3): accDeadband_s, xy, z

### Community 314 - "flight/imu: rollAndPitchInclination_s"
Cohesion: 0.67
Nodes (3): rollAndPitchInclination_s, pitchDeciDegrees, rollDeciDegrees

### Community 315 - "io/rc_controls: channelRange_s"
Cohesion: 0.67
Nodes (3): channelRange_s, endStep, startStep

### Community 316 - "sensors/acceleration: rollAndPitchTrims_s"
Cohesion: 0.67
Nodes (3): rollAndPitchTrims_s, pitch, roll

### Community 32 - "API + drivers: Localisation, Motor"
Cohesion: 0.07
Nodes (12): Localisation_P, motor_direction, Bms_Get(), Localisation_P::init(), localisation_type, ANTICLOCK_WISE, CLOCK_WISE, flightStatusMap_t (+4 more)

### Community 44 - "USB-FS/core: CONTROL_STATE"
Cohesion: 0.18
Nodes (11): CONTROL_STATE, IN_DATA, LAST_IN_DATA, LAST_OUT_DATA, OUT_DATA, PAUSE, SETTING_UP, STALLED (+3 more)

### Community 51 - "drivers + StdPeriph: dma, serial"
Cohesion: 0.06
Nodes (53): ADC_PinMapping, portOptions_t, DMA_ClearFlag(), DMA_ClearITPendingBit(), DMA_Cmd(), DMA_DeInit(), DMA_GetCurrDataCounter(), DMA_GetFlagStatus() (+45 more)

### Community 55 - "io/gps: gpsInitUblox()"
Cohesion: 0.06
Nodes (40): gpsDataNmea_s, gpsInitData_s, sensorsClear(), serialGetBaudRate(), serialPrint(), serialSetBaudRate(), GPS_coord_to_degrees(), gpsInitHardware() (+32 more)

### Community 59 - "drivers/serial: serialPort_s"
Cohesion: 0.12
Nodes (16): serialPort_s, baudRate, callback, identifier, mode, options, rxBuffer, rxBufferHead (+8 more)

### Community 68 - "flight/lowpass: lowpass_s"
Cohesion: 0.14
Nodes (14): lowpass_s, a, af, b, bf, coeff_shift, freq, init (+6 more)

### Community 70 - "io/gps: gpsConfig_s"
Cohesion: 0.17
Nodes (12): gpsConfig_s, ubloxSbas_s, gpsAutoBaud_e, gpsAutoConfig_e, autoBaud, autoConfig, provider, sbasMode (+4 more)

### Community 74 - "API + command: Scheduler-Timer, command"
Cohesion: 0.08
Nodes (36): ScheduledFn, Command_Arm(), ensure_task_init(), Execute_Scheduled(), Interval::check(), Interval::set(), Scheduler_Add(), Scheduler_Cancel() (+28 more)

### Community 75 - "drivers/accgyro_mpu: lpf_e"
Cohesion: 0.06
Nodes (39): accel_fsr_e, clock_sel_e, gyro_fsr_e, lpf_e, mpuConfiguration_s, mpuDetectionResult_s, INV_FSR_16G, INV_FSR_2G (+31 more)

### Community 79 - "io/gps: gpsCoordinateDDDMMmmmm_s"
Cohesion: 0.67
Nodes (3): gpsCoordinateDDDMMmmmm_s, dddmm, mmmm

### Community 80 - "API/Motor: bidirectional_motor"
Cohesion: 0.29
Nodes (7): bidirectional_motor, M1, M2, M5, M6, M7, M8

### Community 84 - "drivers/tof_dev_area: tofDevArea_u"
Cohesion: 0.50
Nodes (3): tofDevArea_u, l0x, l1x

### Community 88 - "drivers/pwm_rx: ppmDevice_s"
Cohesion: 0.08
Nodes (28): ppmDevice_s, timerConfig_s, ppmAvoidPWMTimerClash(), ppmEdgeCallback(), ppmInit(), ppmOverflowCallback(), pwmEdgeCallback(), pwmICConfig() (+20 more)

### Community 90 - "API/XRanging: laser_e"
Cohesion: 0.10
Nodes (28): GpioAlias, LaserMap, adcPinEnabled(), X1Ranging_P::getRange(), X1Ranging_P::getStatus(), X1Ranging_P::init(), X1Ranging_P::initObjectAvoidance(), altPinConflict() (+20 more)

### Community 93 - "rx/rx: crsfFrameStatus()"
Cohesion: 0.13
Nodes (14): rxFailsafeChannelConfiguration_s, rxRuntimeConfig_s, crsfCrc8(), crsfFrameStatus(), crsfSendBatteryFrame(), crsfUnpackChannels(), mode, step (+6 more)

### Community 0 - "flight + io: mixer, mw"
Cohesion: 0.05
Nodes (57): Command_DisArm(), canUseBlackboxWithCurrentConfiguration(), initBlackbox(), command_jump(), command_next(), command_previous(), command_run(), feature() (+49 more)

### Community 10 - "drivers/pwm_output: gpioInit()"
Cohesion: 0.10
Nodes (25): TIM_CtrlPWMOutputs(), i2cInit(), gpioInit(), initInverter(), ledInit(), pwmInit(), isMotorBrushed(), pwmBrushedMotorConfig() (+17 more)

### Community 100 - "sensors/battery: annexCode()"
Cohesion: 0.09
Nodes (44): applyObjectAvoidance(), cmp32(), reset_FSI(), set_FSI(), INA219_ReadBus_mV(), INA219_ReadShunt_10uV(), ina219ReadReg(), limitAltitude() (+36 more)

### Community 102 - "rx/rx: detectAndApplySignalLossBehavio…"
Cohesion: 0.07
Nodes (34): adcGetChannel(), isPPMDataBeingReceived(), isPWMDataBeingReceived(), resetPPMDataReceivedState(), failsafeOnValidDataFailed(), failsafeOnValidDataReceived(), rxMspFrameComplete(), applyRxChannelRangeConfiguraton() (+26 more)

### Community 103 - "io/flashfs: flashfsWriteBuffers()"
Cohesion: 0.17
Nodes (28): blackboxDeviceReserveBufferSpace(), m25p16_readBytes(), flashfsAdvanceTailInBuffer(), flashfsBufferIsEmpty(), flashfsClearBuffer(), flashfsEraseCompletely(), flashfsFlushAsync(), flashfsFlushSync() (+20 more)

### Community 104 - "CMSIS/arm_math: __INLINE"
Cohesion: 0.14
Nodes (28): arm_circularRead_f32(), arm_circularRead_q7(), arm_circularWrite_f32(), arm_circularWrite_q7(), arm_linear_interp_q31(), arm_linear_interp_q7(), clip_q31_to_q15(), clip_q31_to_q7() (+20 more)

### Community 106 - "StdPeriph/flash: writeEEPROM()"
Cohesion: 0.11
Nodes (29): FLASH_ClearFlag(), FLASH_EraseAllPages(), FLASH_ErasePage(), FLASH_GetFlagStatus(), FLASH_GetStatus(), FLASH_HalfCycleAccessCmd(), FLASH_ITConfig(), FLASH_Lock() (+21 more)

### Community 109 - "telemetry/frsky: handleFrSkyTelemetry()"
Cohesion: 0.22
Nodes (27): serialWrite(), GPStoDDDMM_MMMM(), handleFrSkyTelemetry(), hasEnoughTimeLapsedSinceLastTelemetryTransmission(), sendAccel(), sendAmperage(), sendBaro(), sendDataHead() (+19 more)

### Community 112 - "drivers/serial_softserial: serialPort_t"
Cohesion: 0.09
Nodes (33): applyChangedBits(), extractAndStoreRxByte(), isSoftSerialTransmitBufferEmpty(), isTimerPeriodTooLarge(), onSerialTimer(), openSoftSerial(), prepareForNextRxByte(), processRxState() (+25 more)

### Community 113 - "config/config: featureSet()"
Cohesion: 0.67
Nodes (3): featureMask(), featureSet(), cliFeature()

### Community 116 - "telemetry/hott: handleHoTTTelemetry()"
Cohesion: 0.15
Nodes (22): addGPSCoordinates(), handleHoTTTelemetry(), hottEAMUpdateBattery(), hottEAMUpdateBatteryDrawnCapacity(), hottEAMUpdateCurrentMeter(), hottPrepareEAMResponse(), hottPrepareGPSResponse(), hottPrepareMessages() (+14 more)

### Community 117 - "flight: imu, filter"
Cohesion: 0.12
Nodes (14): biquadApply(), biquadInitLPF(), initIMUFilters(), calculateAccZLowPassFilterRCTimeConstant(), calculateThrottleAngleScale(), dcmMulXY(), imuConfigure(), yaw_error_compass() (+6 more)

### Community 118 - "VL53L1X/api_preset_modes: VL53L1_set_preset_mode()"
Cohesion: 0.39
Nodes (18): VL53L1_set_preset_mode(), VL53L1_init_tuning_parm_storage_struct(), VL53L1_preset_mode_low_power_auto_long_ranging(), VL53L1_preset_mode_low_power_auto_ranging(), VL53L1_preset_mode_low_power_auto_short_ranging(), VL53L1_preset_mode_olt(), VL53L1_preset_mode_singleshot_ranging(), VL53L1_preset_mode_standard_ranging() (+10 more)

### Community 12 - "drivers/bus_i2c_stm32f30x: i2cRead_v2()"
Cohesion: 0.22
Nodes (20): I2C_ClearFlag(), I2C_GetFlagStatus(), I2C_ReceiveData(), I2C_SendData(), I2C_TransferHandling(), icp10111_measureStart(), icp10111Detect(), i2cInit() (+12 more)

### Community 125 - "USB-FS: regs, mem"
Cohesion: 0.21
Nodes (13): DataStageIn(), DataStageOut(), PMAToUserBufferCopy(), UserToPMABufferCopy(), GetEPRxAddr(), GetEPRxCount(), GetEPTxAddr(), SetEPTxCount() (+5 more)

### Community 126 - "common + flight: maths, imu"
Cohesion: 0.05
Nodes (49): applyDeadband(), arraySubInt32(), buildRotationMatrix(), devClear(), devPush(), devStandardDeviation(), devVariance(), normalizeV() (+41 more)

### Community 127 - "VL53L0X/api: VL53L0X_DEV"
Cohesion: 0.11
Nodes (47): VL53L0X_apply_offset_adjustment(), VL53L0X_get_offset_calibration_data_micro_meter(), VL53L0X_perform_offset_calibration(), VL53L0X_perform_xtalk_calibration(), VL53L0X_set_offset_calibration_data_micro_meter(), VL53L0X_get_pal_range_status(), VL53L0X_get_total_signal_rate(), VL53L0X_get_total_xtalk_rate() (+39 more)

### Community 128 - "VL53L0X/api_calibration: VL53L0X_WrByte()"
Cohesion: 0.26
Nodes (19): count_enabled_spads(), enable_ref_spads(), enable_spad_bit(), get_next_good_spad(), get_ref_spad_map(), is_aperture(), perform_ref_signal_measurement(), set_ref_spad_map() (+11 more)

### Community 130 - "drivers/paw3903_opticflow: paw3903_read_reg()"
Cohesion: 0.15
Nodes (19): paw3903_check_id(), paw3903_get_mode(), paw3903_get_orientation(), paw3903_get_resolution(), paw3903_init(), paw3903_power_up_reset(), paw3903_read_motion(), paw3903_read_motion_burst() (+11 more)

### Community 132 - "flight/lowpass: lowpassFixed()"
Cohesion: 0.70
Nodes (3): generateLowpassCoeffs2(), lowpassFixed(), lowpass_t

### Community 135 - "API + drivers: Motor, RxConfig"
Cohesion: 0.18
Nodes (16): Motor_Init(), Motor_Set(), Motor_SetDir(), xshutPinWrite(), ESP_WiFi_OFF(), ESP_WiFi_ON(), STM_PB3_ESP_IO14_Init(), digitalHi() (+8 more)

### Community 136 - "sensors/acceleration: flightAccelCalData_T"
Cohesion: 0.50
Nodes (4): resetAccelerometerCalibration(), getAccelerationCalibration(), setAccelerationCalibration(), flightAccelCalData_T

### Community 137 - "CMSIS/cmFunc: __STATIC_INLINE"
Cohesion: 0.17
Nodes (17): __get_APSR(), __get_BASEPRI(), __get_CONTROL(), __get_FAULTMASK(), __get_FPSCR(), __get_IPSR(), __get_MSP(), __get_PRIMASK() (+9 more)

### Community 138 - "CMSIS/cm0: __STATIC_INLINE"
Cohesion: 0.12
Nodes (22): NVIC_ClearPendingIRQ(), NVIC_DisableIRQ(), NVIC_EnableIRQ(), NVIC_GetPendingIRQ(), NVIC_GetPriority(), NVIC_SetPendingIRQ(), NVIC_SetPriority(), NVIC_SystemReset() (+14 more)

### Community 139 - "CMSIS/cmInstr: __STATIC_INLINE"
Cohesion: 0.18
Nodes (19): __CLREX(), __CLZ(), __DMB(), __ISB(), __LDREXB(), __LDREXH(), __LDREXW(), __NOP() (+11 more)

### Community 14 - "telemetry + io: smartport, serial"
Cohesion: 0.14
Nodes (18): serialRead(), serialRxBytesWaiting(), serialSetMode(), gpsEnablePassthrough(), cliGpsPassthrough(), mspSerialRead(), mspSerialRxBytesWaiting(), main() (+10 more)

### Community 140 - "StdPeriph/dac: DAC_TypeDef"
Cohesion: 0.13
Nodes (24): RCC_GetITStatus(), DAC_ClearFlag(), DAC_ClearITPendingBit(), DAC_Cmd(), DAC_DeInit(), DAC_DMACmd(), DAC_DualSoftwareTriggerCmd(), DAC_GetDataOutputValue() (+16 more)

### Community 141 - "API + common: FC-Control-Status, maths"
Cohesion: 0.40
Nodes (4): FlightStatus_Check(), FlightStatus_Get(), leastSignificantBit(), ledStripFlightStatus()

### Community 142 - "flight/failsafe: useFailsafeConfig()"
Cohesion: 0.67
Nodes (3): failsafeReset(), useFailsafeConfig(), failsafeConfig_t

### Community 143 - "CMSIS/gpio: GPIO_DeInit()"
Cohesion: 0.12
Nodes (14): GPIO_DeInit(), GPIO_Init(), GPIO_PinAFConfig(), GPIO_PinLockConfig(), GPIO_ReadInputData(), GPIO_ReadInputDataBit(), GPIO_ReadOutputData(), GPIO_ReadOutputDataBit() (+6 more)

### Community 149 - "CMSIS: sc000, cm0plus"
Cohesion: 0.08
Nodes (37): NVIC_ClearPendingIRQ(), NVIC_DisableIRQ(), NVIC_EnableIRQ(), NVIC_GetPendingIRQ(), NVIC_GetPriority(), NVIC_SetPendingIRQ(), NVIC_SetPriority(), SysTick_Config() (+29 more)

### Community 15 - "VL53L0X/api: VL53L0X_Error"
Cohesion: 0.07
Nodes (56): VL53L0X_get_device_error_string(), VL53L0X_get_limit_check_info(), VL53L0X_get_pal_error_string(), VL53L0X_get_pal_state_string(), VL53L0X_get_range_status_string(), VL53L0X_get_sequence_steps_info(), VL53L0X_CheckAndLoadInterruptSettings(), VL53L0X_EnableInterruptMask() (+48 more)

### Community 150 - "USB-FS/core: NoData_Setup0()"
Cohesion: 0.14
Nodes (18): Data_Setup0(), In0_Process(), NoData_Setup0(), Out0_Process(), Post0_Process(), SetDeviceAddress(), Setup0_Process(), Standard_ClearFeature() (+10 more)

### Community 152 - "StdPeriph/exti: configureMPUDataReadyInterruptH…"
Cohesion: 0.12
Nodes (15): EXTI_ClearITPendingBit(), EXTI_GetFlagStatus(), EXTI_GetITStatus(), EXTI_Init(), EXTI_StructInit(), SYSCFG_EXTILineConfig(), icm20948AccInit(), configureMPUDataReadyInterruptHandling() (+7 more)

### Community 153 - "StdPeriph/hrtim: HRTIM_ADCTriggerConfig()"
Cohesion: 0.11
Nodes (18): HRTIM_ADCTriggerConfig(), HRTIM_BurstModeConfig(), HRTIM_ChopperModeConfig(), HRTIM_DeadTimeConfig(), HRTIM_FaultConfig(), HRTIM_SynchronizationConfig(), HRTIM_TimerEventFilteringConfig(), HRTIM_WaveformCaptureConfig() (+10 more)

### Community 154 - "StdPeriph/tim: ws2811LedStripHardwareInit()"
Cohesion: 0.15
Nodes (23): TIM_BDTRConfig(), TIM_BDTRStructInit(), TIM_DMACmd(), TIM_OC1Init(), TIM_OC1PreloadConfig(), TIM_OC2Init(), TIM_OC2PreloadConfig(), TIM_OC3Init() (+15 more)

### Community 158 - "API + drivers: Serial-IO-Uart, opticflow_paw3903"
Cohesion: 0.17
Nodes (15): getBaud(), Uart_Init(), Uart_Read16(), Uart_Read32(), Uart_Read8(), Uart_rxBytesWaiting(), Uart_txBytesFree(), Uart_Write() (+7 more)

### Community 159 - "StdPeriph/opamp: OPAMP_InitTypeDef"
Cohesion: 0.13
Nodes (9): OPAMP_Cmd(), OPAMP_Init(), OPAMP_StartCalibration(), OPAMP_StructInit(), OPAMP_TimerControlledMuxCmd(), OPAMP_TimerControlledMuxConfig(), OPAMP_VrefConnectADCCmd(), OPAMP_VrefConnectNonInvertingInput() (+1 more)

### Community 16 - "StdPeriph/adc: ADC_TypeDef"
Cohesion: 0.05
Nodes (79): ADC_AnalogWatchdog1SingleChannelConfig(), ADC_AnalogWatchdog1ThresholdsConfig(), ADC_AnalogWatchdog2SingleChannelConfig(), ADC_AnalogWatchdog2ThresholdsConfig(), ADC_AnalogWatchdog3SingleChannelConfig(), ADC_AnalogWatchdog3ThresholdsConfig(), ADC_AnalogWatchdogCmd(), ADC_AutoDelayCmd() (+71 more)

### Community 168 - "drivers/bridge_sc18is602b: sc18_i2c_write()"
Cohesion: 0.20
Nodes (13): sc18_address_cfg(), sc18_clearInterrupt(), sc18_configureSPI(), sc18_enableGPIO(), sc18_i2c_read(), sc18_i2c_write(), sc18_readGPIO(), sc18_setLowPowerMode() (+5 more)

### Community 169 - "drivers/tof_manager_l1x: tofId_e"
Cohesion: 0.23
Nodes (17): l1xBe16(), l1xDecodeResult(), l1xRangeStatus(), tofIdValid(), tofL1xBusError(), tofL1xCheckStale(), tofL1xEntry(), tofL1xLastSampleMs() (+9 more)

### Community 17 - "StdPeriph: pwr, wwdg"
Cohesion: 0.09
Nodes (9): PWR_BackupAccessCmd(), PWR_DeInit(), PWR_GetFlagStatus(), PWR_PVDCmd(), PWR_WakeUpPinCmd(), RCC_APB1PeriphResetCmd(), WWDG_DeInit(), WWDG_GetFlagStatus() (+1 more)

### Community 172 - "drivers: compass_ak09916, opticflow_paw3903"
Cohesion: 0.12
Nodes (25): SPI_Write(), FC_Reboot_Led(), icm20948GyroInit(), icmSelectBank(), sc18_reset(), ak09916Detect(), ak09916Init(), ak09916Read() (+17 more)

### Community 18 - "drivers/timer: timerHardware_t"
Cohesion: 0.12
Nodes (38): TIM_ClearFlag(), TIM_Cmd(), TIM_ITConfig(), ppmInConfig(), pwmInConfig(), serialTimerRxConfig(), serialTimerTxConfig(), configTimeBase() (+30 more)

### Community 180 - "config + io: config, display"
Cohesion: 0.06
Nodes (42): resetUser(), degreesToRadians(), changeControlRateProfile(), changeProfile(), initEEPROM(), latchActiveFeatures(), readEEPROM(), readEEPROMAndNotify() (+34 more)

### Community 183 - "VL53L1X/register_funcs: VL53L1_init_and_start_range()"
Cohesion: 0.16
Nodes (12): VL53L1_init_and_start_range(), VL53L1_i2c_decode_int16_t(), VL53L1_i2c_encode_int16_t(), VL53L1_update_ll_driver_cfg_state(), VL53L1_update_ll_driver_rd_state(), VL53L1_get_customer_nvm_managed(), VL53L1_get_system_control(), VL53L1_i2c_decode_customer_nvm_managed() (+4 more)

### Community 186 - "io + user: oled_display, PlutoPilot"
Cohesion: 0.13
Nodes (22): onLoopFinish(), onLoopStart(), plutoInit(), plutoLoop(), FlightMode_Check(), FlightMode_Set(), status_FSI(), dispArmData() (+14 more)

### Community 19 - "drivers + API: bus_spi, Serial-IO-SPI"
Cohesion: 0.23
Nodes (13): SPI_I2S_GetFlagStatus(), SPI_ReceiveData8(), SPI_SendData8(), getPrescaler(), SPI_Disable(), SPI_Enable(), SPI_Read(), spiGetErrorCounter() (+5 more)

### Community 195 - "StdPeriph/hrtim: HRTIM_OutputConfig()"
Cohesion: 0.19
Nodes (13): HRTIM_CompareUnitConfig(), HRTIM_OutputConfig(), HRTIM_SimpleOCChannelConfig(), HRTIM_SimpleOnePulseChannelConfig(), HRTIM_SimplePWMChannelConfig(), HRTIM_TIM_ResetConfig(), HRTIM_WaveformCompareConfig(), HRTIM_WaveformOutputConfig() (+5 more)

### Community 198 - "common/printf: tfp_format()"
Cohesion: 0.12
Nodes (18): fputc(), init_printf(), printfSupportInit(), _putc(), putchw(), putcp(), setPrintfSerialPort(), tfp_format() (+10 more)

### Community 199 - "drivers/bus_i2c_soft: I2C_delay()"
Cohesion: 0.44
Nodes (11): I2C_Ack(), I2C_delay(), I2C_NoAck(), I2C_ReceiveByte(), I2C_SendByte(), I2C_Start(), I2C_Stop(), I2C_WaitAck() (+3 more)

### Community 20 - "StdPeriph/tim: TIM_ICInit()"
Cohesion: 0.16
Nodes (17): TI1_Config(), TI2_Config(), TI3_Config(), TI4_Config(), TIM_ICInit(), TIM_ICStructInit(), TIM_PWMIConfig(), TIM_SetCounter() (+9 more)

### Community 200 - "drivers/display_ug2864hsweg01: i2c_OLED_send_cmd()"
Cohesion: 0.32
Nodes (13): bitreverse8(), i2c_OLED_clear_display(), i2c_OLED_clear_display_quick(), i2c_OLED_draw_bitmap(), i2c_OLED_draw_rect(), i2c_OLED_send_byte(), i2c_OLED_send_bytes(), i2c_OLED_send_changed_bytes() (+5 more)

### Community 203 - "CMSIS/arm_math: float32_t"
Cohesion: 0.17
Nodes (12): arm_bilinear_interp_f32(), arm_clarke_f32(), arm_inv_clarke_f32(), arm_inv_park_f32(), arm_linear_interp_f32(), arm_park_f32(), arm_pid_f32(), arm_sqrt_f32() (+4 more)

### Community 204 - "StdPeriph/hrtim: HRTIM_TimingUnitBase_Config()"
Cohesion: 0.32
Nodes (12): HRTIM_MasterBase_Config(), HRTIM_MasterWaveform_Config(), HRTIM_SimpleBase_Init(), HRTIM_SimpleCapture_Init(), HRTIM_SimpleOC_Init(), HRTIM_SimpleOnePulse_Init(), HRTIM_SimplePWM_Init(), HRTIM_TimingUnitBase_Config() (+4 more)

### Community 21 - "VL53L1X/core: VL53L1_WrByte()"
Cohesion: 0.05
Nodes (48): VL53L1_run_device_test(), VL53L1_run_ref_spad_char(), VL53L1_run_spad_rate_map(), VL53L1_set_GPIO_interrupt_config(), VL53L1_init_ssc_config_struct(), VL53L1_nvm_enable(), VL53L1_calc_pll_period_us(), VL53L1_decode_row_col() (+40 more)

### Community 215 - "VL53L1X: api_core, core"
Cohesion: 0.24
Nodes (8): VL53L1_copy_sys_and_core_results_to_range_results(), VL53L1_get_device_results(), VL53L1_get_measurement_results(), VL53L1_print_range_results(), VL53L1_check_ll_driver_rd_state(), VL53L1_low_power_auto_setup_manual_calibration(), VL53L1_low_power_auto_update_DSS(), VL53L1_range_results_t

### Community 22 - "blackbox/blackbox: blackboxWrite()"
Cohesion: 0.19
Nodes (22): blackboxIsOnlyLoggingIntraframes(), blackboxLogIteration(), blackboxShouldLogPFrame(), blackboxWriteMainStateArrayUsingAveragePredictor(), blackboxWrite(), blackboxWriteSigned16VBArray(), blackboxWriteSignedVB(), blackboxWriteSignedVBArray() (+14 more)

### Community 23 - "io/ledstrip: setLedHsv()"
Cohesion: 0.26
Nodes (10): setLedHsv(), setStripColor(), setStripColors(), applyDefaultColors(), applyDirectionalModeColor(), applyLedModeLayer(), applyQuadrantColor(), ledStripInit() (+2 more)

### Community 232 - "CMSIS/arm_math: __QADD()"
Cohesion: 0.32
Nodes (8): arm_clarke_q31(), arm_inv_clarke_q31(), arm_inv_park_q31(), arm_park_q31(), arm_recip_q31(), clip_q63_to_q31(), __QADD(), __QSUB()

### Community 233 - "CMSIS/arm_math: arm_pid_q15()"
Cohesion: 0.25
Nodes (8): arm_pid_q15(), clip_q63_to_q15(), mult32x64(), __SMLALD(), __SMLALDX(), __SMUAD(), arm_pid_instance_q15, q63_t

### Community 234 - "VL53L0X/api: VL53L0X_PerformSingleRangingMea…"
Cohesion: 0.24
Nodes (14): VL53L0X_ClearInterruptMask(), VL53L0X_GetDeviceMode(), VL53L0X_GetGpioConfig(), VL53L0X_GetRangingMeasurementData(), VL53L0X_PerformSingleMeasurement(), VL53L0X_PerformSingleRangingMeasurement(), VL53L0X_SetGpioConfig(), VL53L0X_StartMeasurement() (+6 more)

### Community 24 - "API/RGB-LED: RGB_UpdateAnimation()"
Cohesion: 0.14
Nodes (21): hsvToRgb(), mapIndex(), RGB_Clear(), RGB_Control(), RGB_FillColor(), RGB_Release(), RGB_SetColor(), RGB_SetColorAll() (+13 more)

### Community 25 - "flight/altitudehold: Estimate_Get()"
Cohesion: 0.06
Nodes (28): userRollPitchRate(), DesiredAngle_setRelative(), DesiredPosition_setRelative(), getDesiredAngle(), getDesiredPositions(), getDesiredRate(), setDesiredAngle(), setDesiredPosition() (+20 more)

### Community 251 - "CMSIS/arm_math: q15_t"
Cohesion: 0.33
Nodes (6): arm_circularRead_q15(), arm_circularWrite_q15(), arm_linear_interp_q15(), arm_recip_q15(), __CLZ(), q15_t

### Community 252 - "StdPeriph/hrtim: HRTIM_ExternalEventConfig()"
Cohesion: 0.40
Nodes (6): HRTIM_CaptureUnitConfig(), HRTIM_EventConfig(), HRTIM_ExternalEventConfig(), HRTIM_SimpleCaptureChannelConfig(), HRTIM_BasicCaptureChannelCfgTypeDef, HRTIM_EventCfgTypeDef

### Community 254 - "flight/mixer: mixerInit()"
Cohesion: 0.29
Nodes (4): mixerInit(), mixerLoadMix(), servoMixerLoadMix(), cliServoMix()

### Community 26 - "StdPeriph/rcc: RCC_APB2PeriphResetCmd()"
Cohesion: 0.05
Nodes (15): RCC_AHBPeriphClockCmd(), RCC_AHBPeriphResetCmd(), RCC_APB1PeriphClockCmd(), RCC_APB2PeriphClockCmd(), RCC_APB2PeriphResetCmd(), RCC_BackupResetCmd(), RCC_ClockSecuritySystemCmd(), RCC_GetClocksFreq() (+7 more)

### Community 27 - "io/ledstrip: updateLedStrip()"
Cohesion: 0.20
Nodes (11): scaleRange(), getLedHsv(), scaleLedValue(), failsafeIsActive(), applyLedAnimationLayer(), applyLedIndicatorLayer(), applyLedThrottleLayer(), applyLedThrustRingLayer() (+3 more)

### Community 28 - "sensors/initialisation: sensorsAutodetectmpu()"
Cohesion: 0.08
Nodes (24): resetSensorAlignment(), sensorsSet(), icm20948AccDetect(), icm20948GyroDetect(), detectMpu(), mpuReadRegisterI2C(), mpuWriteRegisterI2C(), detectAcc() (+16 more)

### Community 29 - "io/oled_display: Oled_DrawHLine()"
Cohesion: 0.17
Nodes (27): Oled_Number(), Oled_Text(), clampPupil(), drawCornerArcs(), drawJoystick(), mapStick(), Oled_display_Clear(), Oled_DrawBoxyEye() (+19 more)

### Community 30 - "StdPeriph/rtc: ErrorStatus"
Cohesion: 0.06
Nodes (37): RTC_AlarmCmd(), RTC_AlarmStructInit(), RTC_Bcd2ToByte(), RTC_BypassShadowCmd(), RTC_ByteToBcd2(), RTC_CalibOutputCmd(), RTC_DateStructInit(), RTC_DeInit() (+29 more)

### Community 31 - "CMSIS/arm_math: arm_bilinear_interp_q15()"
Cohesion: 0.04
Nodes (54): arm_bilinear_interp_q15(), arm_pid_q31(), arm_bilinear_interp_instance_q15, arm_biquad_cas_df1_32x64_ins_q31, arm_biquad_cascade_df2T_instance_f32, arm_biquad_casd_df1_inst_f32, arm_biquad_casd_df1_inst_q15, arm_biquad_casd_df1_inst_q31 (+46 more)

### Community 33 - "StdPeriph/i2c: FunctionalState"
Cohesion: 0.07
Nodes (48): I2C_10BitAddressHeaderCmd(), I2C_10BitAddressingModeCmd(), I2C_AcknowledgeConfig(), I2C_AutoEndCmd(), I2C_CalculatePEC(), I2C_ClearITPendingBit(), I2C_ClockTimeoutCmd(), I2C_Cmd() (+40 more)

### Community 34 - "StdPeriph/usart: USART_TypeDef"
Cohesion: 0.06
Nodes (58): USART_AddressDetectionConfig(), USART_AutoBaudRateCmd(), USART_AutoBaudRateConfig(), USART_ClearFlag(), USART_ClearITPendingBit(), USART_ClockInit(), USART_ClockStructInit(), USART_Cmd() (+50 more)

### Community 35 - "io/serial_cli: cliPrint()"
Cohesion: 0.13
Nodes (22): featureConfigured(), handleOneshotFeatureChangeOnRestart(), sensorsMask(), pwmShutdownPulsesForAllMotors(), systemReset(), systemResetToBootloader(), StopPwmAllMotors(), cliDefaults() (+14 more)

### Community 36 - "CMSIS/cm3: __INLINE"
Cohesion: 0.08
Nodes (41): __CLREX(), __get_BASEPRI(), __set_BASEPRI(), __get_PRIMASK(), __set_PRIMASK(), __get_FAULTMASK(), __set_FAULTMASK(), __get_CONTROL() (+33 more)

### Community 37 - "StdPeriph/gpio: GPIO_TypeDef"
Cohesion: 0.18
Nodes (16): GPIO_DeInit(), GPIO_Init(), GPIO_PinAFConfig(), GPIO_PinLockConfig(), GPIO_ReadInputData(), GPIO_ReadInputDataBit(), GPIO_ReadOutputData(), GPIO_ReadOutputDataBit() (+8 more)

### Community 38 - "USB-FS/regs: Virtual_Com_Port_Reset()"
Cohesion: 0.04
Nodes (13): Clear_Status_Out(), GetENDPOINT(), GetEPDblBufDir(), SetBTABLE(), SetDouBleBuffEPStall(), SetEPRxAddr(), SetEPRxStatus(), SetEPRxValid() (+5 more)

### Community 39 - "VL53L1X: ll_def, api_debug"
Cohesion: 0.06
Nodes (44): VL53L1_run_offset_calibration(), ComputeRQL(), VL53L1_data_init(), VL53L1_get_version(), VL53L1_get_additional_data(), VL53L1_print_additional_data(), VL53L1_print_additional_offset_cal_data(), VL53L1_print_cal_peak_rate_map() (+36 more)

### Community 4 - "VL53L1X: api, api_core"
Cohesion: 0.05
Nodes (119): ChangePresetMode(), CheckValidRectRoi(), ComputeDevicePresetMode(), ConvertStatusLite(), VL53L1_clear_interrupt_and_enable_next_range(), VL53L1_disable_xtalk_compensation(), VL53L1_enable_xtalk_compensation(), VL53L1_get_calibration_repeat_period() (+111 more)

### Community 40 - "sensors/barometer: apmBaroCalibrate()"
Cohesion: 0.19
Nodes (20): apmBaroCalibrate(), apmBaroCalibrationRead(), apmBaroRead(), apmBaroUpdate(), baroCalibrate(), baroCompensationPa(), baroInit(), baroResetGroundLevel() (+12 more)

### Community 41 - "flight/navigation: onGpsNewData()"
Cohesion: 0.11
Nodes (37): get_PIDProfile(), set_PIDProfile(), cos_approx(), sin_approx(), resetGpsProfile(), imuCalculateHeading(), check_missed_wp(), get_D() (+29 more)

### Community 42 - "io/serial_msp: processInCommand()"
Cohesion: 0.15
Nodes (26): gpio_set_mode(), usb1WireInitialize(), usb1WirePassthrough(), findBoxByActiveBoxId(), findBoxByPermenantId(), headSerialError(), headSerialReply(), headSerialResponse() (+18 more)

### Community 43 - "CMSIS/cm4_simd: __STATIC_INLINE"
Cohesion: 0.08
Nodes (49): __QADD(), __QADD16(), __QADD8(), __QASX(), __QSAX(), __QSUB(), __QSUB16(), __QSUB8() (+41 more)

### Community 45 - "drivers/tof_manager: xRangingInit()"
Cohesion: 0.15
Nodes (19): xshutPinInit(), rangingFamily(), xRangingInit(), updateSpiOpticFlow(), vl53l0xProbe(), tofIdValid(), tofManagerBringUp(), tofManagerEntry() (+11 more)

### Community 46 - "config + sensors: config, barometer"
Cohesion: 0.10
Nodes (17): featureClearAll(), resetBarometerConfig(), resetBatteryConfig(), resetConf(), resetControlRateConfig(), resetEscAndServoConfig(), resetFlight3DConfig(), resetMixerConfig() (+9 more)

### Community 47 - "blackbox/blackbox: startBlackbox()"
Cohesion: 0.25
Nodes (7): blackboxBuildConditionCache(), blackboxCheckAndLogArmingBeep(), gcd(), startBlackbox(), testBlackboxConditionUncached(), validateBlackboxConfig(), getArmingBeepTimeMicros()

### Community 48 - "io/serial_cli: cliDump()"
Cohesion: 0.23
Nodes (24): cliAdjustmentRange(), cliAux(), cliColor(), cliDump(), cliFlashRead(), cliLed(), cliMap(), cliMotor() (+16 more)

### Community 49 - "API/Oled: Oled_DrawLine()"
Cohesion: 0.08
Nodes (24): Oled_Circle(), Oled_CircleOutline(), Oled_DeadEyes(), Oled_Erase(), Oled_Joysticks(), Oled_Line(), Oled_PitchLine(), Oled_Pixel() (+16 more)

### Community 5 - "VL53L1X/register_funcs: VL53L1_ReadMulti()"
Cohesion: 0.06
Nodes (86): VL53L1_disable_firmware(), VL53L1_enable_firmware(), VL53L1_i2c_decode_int32_t(), VL53L1_i2c_decode_uint16_t(), VL53L1_i2c_decode_uint32_t(), VL53L1_i2c_encode_int32_t(), VL53L1_i2c_encode_uint16_t(), VL53L1_i2c_encode_uint32_t() (+78 more)

### Community 50 - "API/RxConfig: Receiver_Aux_Config()"
Cohesion: 0.32
Nodes (12): plutoRxConfig(), Receiver_Aux_Config(), Receiver_Config_Arm(), Receiver_Config_Mode_Angle(), Receiver_Config_Mode_Baro(), Receiver_Config_Mode_Dev(), Receiver_Config_Mode_HeadFree(), Receiver_Config_Mode_Mag() (+4 more)

### Community 52 - "flight: altitudehold, posEstimate"
Cohesion: 0.15
Nodes (17): addHistPositionBaseEstZ(), AltRst(), apmCalculateEstimatedAltitude(), calculateEstimatedAltitude(), checkReading(), kalmanFilterInit(), kalmanFilterUpdate(), imuResetAccelerationSum() (+9 more)

### Community 53 - "sensors/acceleration: updateAccelerationReadings()"
Cohesion: 0.12
Nodes (16): Set_LED(), resetAccelerometerTrims(), flightStatusIndicator(), ledOperator(), applyAccelerationTrims(), calibrate_accel(), calibrate_find_delta(), calibrate_reset_matrices() (+8 more)

### Community 54 - "flight + API: pid, acceleration"
Cohesion: 0.07
Nodes (50): userAngleTarget(), setDefault_PIDProfile(), setUserLoopFrequency(), Command_Flip(), Command_TakeOff(), landingInProgress(), Peripheral_Write(), Servo_Write() (+42 more)

### Community 56 - "vcp/hw_config: usbVcpOpen()"
Cohesion: 0.16
Nodes (9): USB_Init(), usbVcpOpen(), usbVcpWrite(), Get_SerialNum(), IntToUnicode(), Set_System(), Set_USBClock(), usbIsConfigured() (+1 more)

### Community 57 - "StdPeriph/hrtim: HRTIM_TypeDef"
Cohesion: 0.08
Nodes (46): HRTIM_BurstDMAConfig(), HRTIM_BurstModeCtl(), HRTIM_ClearCommonFlag(), HRTIM_ClearCommonITPendingBit(), HRTIM_ClearFlag(), HRTIM_ClearITPendingBit(), HRTIM_DeInit(), HRTIM_DLLCalibrationStart() (+38 more)

### Community 58 - "rx + API: API-Utils, statusindicator"
Cohesion: 0.22
Nodes (10): resetUserRCflag(), userRCrelease(), micros(), filterServos(), warningLedRefresh(), warningLedUpdate(), crsfDataReceive(), sbusDataReceive() (+2 more)

### Community 6 - "API/X1Ranging: x1RangingInit()"
Cohesion: 0.13
Nodes (19): altRequest(), isLedUsable(), isValidGpio(), ledInit(), ledWrite(), X1Ranging_P::disableOA(), X1Ranging_P::initAlt(), X1Ranging_P::isTriggered() (+11 more)

### Community 60 - "telemetry: frsky, hott"
Cohesion: 0.18
Nodes (15): closeSerialPort(), mspReleasePortIfAllocated(), checkFrSkyTelemetryState(), configureFrSkyTelemetryPort(), freeFrSkyTelemetryPort(), checkHoTTTelemetryState(), configureHoTTTelemetryPort(), freeHoTTTelemetryPort() (+7 more)

### Community 61 - "io/serial_cli: cliMotorMix()"
Cohesion: 0.23
Nodes (12): fastA2F(), ftoa(), itoa(), getCurrentControlRateProfile(), cliGet(), cliMotorMix(), cliPrintVar(), cliSet() (+4 more)

### Community 62 - "common/atomic: __basepriSetMemRetVal()"
Cohesion: 0.38
Nodes (4): __basepriSetMemRetVal(), __basepriSetRetVal(), __set_BASEPRI_MAX(), __set_BASEPRI_MAX_nb()

### Community 63 - "telemetry: serial, frsky"
Cohesion: 0.24
Nodes (6): determinePortSharing(), initFrSkyTelemetry(), initHoTTTelemetry(), initMSPTelemetry(), initSmartPortTelemetry(), telemetryInit()

### Community 64 - "flight/altitudehold: checkReadingFor()"
Cohesion: 0.28
Nodes (9): altLaserFusion(), checkBaro(), checkReadingFor(), correctedWithBaro(), getFrontHistPositionBaseEstZ(), isPositionBaseQueueIsFull(), apmBaroCalculateAltitude(), baroCalculateAltitude() (+1 more)

### Community 65 - "rx/xbus: xBusDataReceive()"
Cohesion: 0.43
Nodes (6): xBusCRC16(), xBusDataReceive(), xBusReadRawRC(), xBusRj01CRC8(), xBusUnpackModeBFrame(), xBusUnpackRJ01Frame()

### Community 66 - "blackbox: blackbox_io, blackbox"
Cohesion: 0.10
Nodes (23): blackboxAdvanceIterationTimers(), blackboxLogEvent(), blackboxSetState(), blackboxShouldLogIFrame(), blackboxWriteSysinfo(), finishBlackbox(), handleBlackbox(), blackboxDeviceClose() (+15 more)

### Community 67 - "drivers/light_ws2811strip: ws2811UpdateStrip()"
Cohesion: 0.17
Nodes (11): RGB_Init(), hsvToRgb24(), fastUpdateLEDDMABuffer(), isWS2811LedStripReady(), ws2811LedStripSetPin(), updateLEDDMABuffer(), ws2811LedStripInit(), ws2811UpdateStrip() (+3 more)

### Community 69 - "drivers/flash_m25p16: m25p16_waitForReady()"
Cohesion: 0.18
Nodes (17): m25p16_eraseCompletely(), m25p16_eraseSector(), m25p16_getGeometry(), m25p16_init(), m25p16_isReady(), m25p16_pageProgram(), m25p16_pageProgramBegin(), m25p16_pageProgramContinue() (+9 more)

### Community 7 - "VL53L0X/api: VL53L0X_RdByte()"
Cohesion: 0.17
Nodes (19): VL53L0X_device_read_strobe(), VL53L0X_get_info_from_device(), VL53L0X_measurement_poll_for_completion(), VL53L0X_check_part_used(), VL53L0X_get_device_info(), VL53L0X_GetDeviceInfo(), VL53L0X_GetFractionEnable(), VL53L0X_GetInterruptMaskStatus() (+11 more)

### Community 71 - "CMSIS/rcc: RCC_GetFlagStatus()"
Cohesion: 0.05
Nodes (15): RCC_AHBPeriphClockCmd(), RCC_APB1PeriphClockCmd(), RCC_APB1PeriphResetCmd(), RCC_APB2PeriphClockCmd(), RCC_APB2PeriphResetCmd(), RCC_BackupResetCmd(), RCC_ClockSecuritySystemCmd(), RCC_GetClocksFreq() (+7 more)

### Community 72 - "VL53L1X: wait, platform"
Cohesion: 0.10
Nodes (22): VL53L1_software_reset(), VL53L1_init_ll_driver_state(), VL53L1_is_firmware_ready_silicon(), VL53L1_is_boot_complete(), VL53L1_is_firmware_ready(), VL53L1_is_new_data_ready(), VL53L1_poll_for_boot_completion(), VL53L1_poll_for_firmware_ready() (+14 more)

### Community 76 - "io/oled_display: Oled_Is_Mode()"
Cohesion: 0.29
Nodes (6): Oled_Is_Mode(), Oled_Mode(), Oled_IsSystemMode(), Oled_IsUserMode(), Oled_SystemMode(), Oled_UserMode()

### Community 77 - "StdPeriph/can: CAN_TypeDef"
Cohesion: 0.11
Nodes (29): CAN_CancelTransmit(), CAN_ClearFlag(), CAN_ClearITPendingBit(), CAN_DBGFreeze(), CAN_DeInit(), CAN_FIFORelease(), CAN_FilterInit(), CAN_GetFlagStatus() (+21 more)

### Community 78 - "VL53L1X/def: ConvertModeFromLLD()"
Cohesion: 0.06
Nodes (30): ConvertModeFromLLD(), ConvertModeToLLD(), VL53L1_get_limit_check_info(), VL53L1_get_pal_state_string(), VL53L1_get_range_status_string(), VL53L1_get_sequence_steps_info(), VL53L1_GetLimitCheckInfo(), VL53L1_GetPalErrorString() (+22 more)

### Community 8 - "StdPeriph/tim: TIM_TypeDef"
Cohesion: 0.05
Nodes (82): TIM_ARRPreloadConfig(), TIM_Break1Cmd(), TIM_Break1Config(), TIM_Break2Cmd(), TIM_Break2Config(), TIM_CCPreloadControl(), TIM_CCxCmd(), TIM_CCxNCmd() (+74 more)

### Community 81 - "StdPeriph/comp: COMP_InitTypeDef"
Cohesion: 0.22
Nodes (6): COMP_Cmd(), COMP_Init(), COMP_StructInit(), COMP_SwitchCmd(), COMP_WindowCmd(), COMP_InitTypeDef

### Community 82 - "StdPeriph/spi: SPI_TypeDef"
Cohesion: 0.10
Nodes (38): I2S_Cmd(), I2S_FullDuplexConfig(), I2S_Init(), I2S_StructInit(), SPI_BiDirectionalLineConfig(), SPI_CalculateCRC(), SPI_Cmd(), SPI_CRCLengthConfig() (+30 more)

### Community 83 - "VL53L0X/api_core: get_sequence_step_timeout()"
Cohesion: 0.16
Nodes (28): get_sequence_step_timeout(), set_sequence_step_timeout(), VL53L0X_calc_dmax(), VL53L0X_calc_macro_period_ps(), VL53L0X_calc_sigma_estimate(), VL53L0X_calc_timeout_mclks(), VL53L0X_calc_timeout_us(), VL53L0X_decode_timeout() (+20 more)

### Community 87 - "API: XRanging, Peripheral-GPIO"
Cohesion: 0.19
Nodes (10): Peripheral_Init(), Peripheral_Read(), Peripheral_Write(), Peripheral_Init(), x1RangingAltXshut(), xRangingAdcClaimed(), xRangingAltXshut(), xRangingGpioClaimed() (+2 more)

### Community 89 - "API/Debugging: debugPrint()"
Cohesion: 0.48
Nodes (7): boundedLength(), debugPrint(), debugPrintTagText(), Monitor_Print(), Monitor_Println(), serializeString(), serialize8Debug()

### Community 9 - "rx + io: rx, serial"
Cohesion: 0.10
Nodes (39): blackboxDeviceOpen(), featureClear(), resetSerialConfig(), validateAndFixConfig(), ppmRead(), pwmRead(), gpsInit(), cliInit() (+31 more)

### Community 91 - "CMSIS/cm4: __STATIC_INLINE"
Cohesion: 0.09
Nodes (30): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), NVIC_DisableIRQ(), NVIC_EnableIRQ(), NVIC_EncodePriority() (+22 more)

### Community 92 - "io/beeper: cliPlaySound()"
Cohesion: 0.40
Nodes (4): beeperModeForTableIndex(), beeperNameForTableIndex(), beeperTableEntryCount(), cliPlaySound()

### Community 94 - "CMSIS/cm3: __STATIC_INLINE"
Cohesion: 0.10
Nodes (29): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), NVIC_DisableIRQ(), NVIC_EnableIRQ(), NVIC_EncodePriority() (+21 more)

### Community 95 - "CMSIS/sc300: __STATIC_INLINE"
Cohesion: 0.10
Nodes (29): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), NVIC_DisableIRQ(), NVIC_EnableIRQ(), NVIC_EncodePriority() (+21 more)

### Community 96 - "vcp: stm32_it, pwr"
Cohesion: 0.08
Nodes (6): Leave_LowPowerMode(), USB_LP_CAN1_RX0_IRQHandler(), USB_Istr(), Resume(), Resume_Init(), Suspend()

### Community 97 - "VL53L0X: i2c_platform, platform"
Cohesion: 0.08
Nodes (20): VL53L0X_load_tuning_settings(), VL53L0X_read_byte(), VL53L0X_read_multi(), VL53L0X_read_word(), VL53L0X_write_byte(), VL53L0X_write_dword(), VL53L0X_write_multi(), VL53L0X_write_word() (+12 more)

### Community 98 - "io/ledstrip: reevalulateLedConfig()"
Cohesion: 0.21
Nodes (9): applyDefaultLedStripConfig(), determineLedStripDimensions(), determineOrientationLimits(), generateLedConfig(), ledStripEnable(), parseColor(), parseLedStripConfig(), reevalulateLedConfig() (+1 more)

### Community 99 - "io/display: updateDisplay()"
Cohesion: 0.15
Nodes (30): tfp_sprintf(), getCurrentProfile(), i2c_OLED_send_char(), i2c_OLED_send_string(), i2c_OLED_set_line(), i2c_OLED_set_xy(), displayInit(), displaySetNextPageChangeAt() (+22 more)

### Community 243 - "StdPeriph/gpio: BitAction"
Cohesion: 0.29
Nodes (6): BitAction, GPIO_InitTypeDef, GPIOMode_TypeDef, GPIOOType_TypeDef, GPIOPuPd_TypeDef, GPIOSpeed_TypeDef

## Knowledge Gaps
- **631 isolated node(s):** `lastIndex`, `lastIndex`, `portConfigs`, `reboot_character`, `blackbox_baudrateIndex` (+626 more)
  These have ≤1 connection - possible missing edges. (Counts symbols only; 1300 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **16 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `FunctionalState` connect `StdPeriph/i2c: FunctionalState` to `StdPeriph/tim: TIM_TypeDef`, `drivers/pwm_output: gpioInit()`, `vcp/prop: RESULT`, `StdPeriph/dac: DAC_TypeDef`, `CMSIS/stm32f30x: IRQn`, `CMSIS/gpio: GPIO_DeInit()`, `StdPeriph/adc: ADC_TypeDef`, `StdPeriph: pwr, wwdg`, `drivers/timer: timerHardware_t`, `StdPeriph/rcc: RCC_APB2PeriphResetCmd()`, `StdPeriph/tim: ws2811LedStripHardwareInit()`, `StdPeriph/rtc: ErrorStatus`, `StdPeriph/opamp: OPAMP_InitTypeDef`, `StdPeriph/usart: USART_TypeDef`, `StdPeriph/crc: CRC_ReverseOutputDataCmd()`, `drivers + StdPeriph: dma, serial`, `StdPeriph/hrtim: HRTIM_TypeDef`, `CMSIS/rcc: RCC_GetFlagStatus()`, `StdPeriph/can: CAN_TypeDef`, `StdPeriph/comp: COMP_InitTypeDef`, `StdPeriph/spi: SPI_TypeDef`, `StdPeriph/flash: writeEEPROM()`?**
  _High betweenness centrality (0.061) - this node is a cross-community bridge._
- **What connects `lastIndex`, `lastIndex`, `portConfigs` to the rest of the system?**
  _631 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `io/serial: serialPortFunction_e` be split into smaller, more focused modules?**
  _Cohesion score 0.07682926829268293 - nodes in this community are weakly interconnected._
- **Why does `VL53L1_Error` connect `VL53L1X: api, api_core` to `VL53L1X/register_funcs: VL53L1_ReadMulti()`, `VL53L1X: ll_def, api_debug`, `VL53L1X: wait, platform`, `drivers/tof_manager_l1x: tofId_e`, `VL53L1X/def: ConvertModeFromLLD()`, `VL53L1X/core: VL53L1_WrByte()`, `VL53L1X/api_preset_modes: VL53L1_set_preset_mode()`, `VL53L1X/register_funcs: VL53L1_init_and_start_range()`, `VL53L1X: api_core, core`?**
  _High betweenness centrality (0.045) - this node is a cross-community bridge._
- **Should `USB-FS/core: DEVICE_INFO` be split into smaller, more focused modules?**
  _Cohesion score 0.07389162561576355 - nodes in this community are weakly interconnected._
- **Why does `TIM_TypeDef` connect `StdPeriph/tim: TIM_TypeDef` to `drivers/pwm_output: gpioInit()`, `CMSIS/stm32f30x: IRQn`, `drivers/timer: timerHardware_t`, `StdPeriph/tim: TIM_ICInit()`, `drivers/pwm_rx: ppmDevice_s`, `StdPeriph/tim: ws2811LedStripHardwareInit()`?**
  _High betweenness centrality (0.035) - this node is a cross-community bridge._
- **Should `vcp/prop: RESULT` be split into smaller, more focused modules?**
  _Cohesion score 0.09538461538461539 - nodes in this community are weakly interconnected._