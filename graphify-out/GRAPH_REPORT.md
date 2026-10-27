# Graph Report - MagisV2  (2026-10-27)

## Corpus Check
- 430 files · ~827,973 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 5110 nodes · 13985 edges · 251 communities (233 shown, 18 thin omitted)
- Extraction: 85% EXTRACTED · 15% INFERRED · 0% AMBIGUOUS · INFERRED: 2133 edges (avg confidence: 0.85)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `a81c180d`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- USB-FS/core: DEVICE_INFO
- drivers/sc18is602b: SC18IS602B
- io/serial: serialPortFunction_e
- drivers/serial: serialPort_s
- flight/mixer: mixerMode
- io/serial_msp: mspPort_s
- flight/altitudehold: AltTofL1x
- flight/altitudehold: AltTofL0x
- io/rc_controls: adjustmentRange_s
- USB-FS/core: RESULT
- rx/rx: rxConfig_s
- drivers/barometer: baro_s
- StdPeriph/crc: IRQn
- blackbox/blackbox_fielddefs: flightLogEventData_u
- flight/failsafe: failsafeState_s
- flight/pid: pidProfile_s
- drivers/timer: timerHardware_t
- drivers/pwm_mapping: drv_pwm_config_s
- blackbox/blackbox: blackboxSlowState_s
- blackbox/blackbox: blackboxMainState_s
- flight/lowpass: lowpass_s
- drivers + API: serial, system
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
- API/X1Ranging: X1Ranging_P
- API/XRanging: XRanging_P
- blackbox/blackbox_fielddefs: blackboxDeltaFieldDefinition_s
- flight/mixer: servoParam_s
- io/rc_controls: controlRateConfig_s
- API/ObjectAvoidanceCore: oaCoreState
- blackbox/blackbox_fielddefs: FlightLogFieldEncoding
- flight/mixer: servoMixer_s
- flight/navigation: gpsProfile_s
- sensors/battery: batteryConfig_s
- API/Motor: bidirectional_motor
- API/Scheduler-Timer: Interval
- API/XRanging: laser_sensors
- blackbox/blackbox_fielddefs: blackboxConditionalFieldDefinit…
- flight/imu: imuRuntimeConfig_s
- io/gps: gpsDataNmea_s
- API/XRanging: laser_status
- blackbox/blackbox_fielddefs: blackboxSimpleFieldDefinition_s
- common/maths: stdev_s
- drivers/sound_beeper: beeperConfig_s
- flight/failsafe: failsafeConfig_s
- blackbox/blackbox_fielddefs: flightLogEvent_inflightAdjustme…
- drivers/adc: adc_config_s
- drivers/adc: drv_adc_config_s
- drivers/tof_dev_area: tofDevArea_u
- flight/mixer: flight3DConfig_s
- flight/mixer: mixer_s
- flight/mixer: motorMixer_s
- io/rc_controls: adjustmentConfig_s
- io/rc_controls: adjustmentConfig_u
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
- flight + io: pid, rc_controls
- io/serial_msp: box_e
- sensors/sensors: int16_flightDynamicsTrims_s
- blackbox/blackbox_fielddefs: flightLogEvent_loggingResume_s
- blackbox/blackbox_fielddefs: FlightLogFieldSign
- drivers/pwm_mapping: pwmOutputConfiguration_s
- flight/imu: accDeadband_s
- flight/imu: rollAndPitchInclination_s
- io/rc_controls: channelRange_s
- sensors/acceleration: rollAndPitchTrims_s
- flight/mixer: airplaneConfig_s
- io/gimbal: gimbalConfig_s
- io/rc_controls: adjustmentSelectConfig_s
- io/rc_controls: adjustmentStepConfig_s
- sensors/gyro: gyroConfig_s
- drivers + StdPeriph: dma, serial
- API/Peripheral-ADC: APIAdcInit()
- drivers/timer: timerCCHandlerRec_t
- io + rx: gps, rx
- API/Motor: Rev_Motor_Gpio
- drivers/pwm_mapping: sonarGPIOConfig_s
- flight/motor: Rev_Motor_Gpio
- API/Localisation: Localisation_P
- API + command: Scheduler-Timer, command
- drivers/accgyro_mpu: lpf_e
- API/Motor: motor_direction
- io/gps: gpsCoordinateDDDMMmmmm_s
- drivers/pwm_rx: ppmDevice_s
- API/XRanging: LaserMap
- io + flight: beeper, failsafe
- rx: rx, gps
- telemetry: frsky, hott
- sensors/battery: BMS_Update()
- rx/rx: detectAndApplySignalLossBehavio…
- io: flashfs, serial_msp
- CMSIS/arm_math: __INLINE
- StdPeriph/flash: FLASH_WaitForLastOperation()
- telemetry/frsky: handleFrSkyTelemetry()
- API + flight: altitudehold, FC-Config-Setpoints
- io/ledstrip: updateLedStrip()
- drivers/serial_softserial: openSoftSerial()
- telemetry + io: hott, smartport
- API/FC-Control-Command: Command_Flip()
- VL53L1X/api_preset_modes: VL53L1_set_preset_mode()
- VL53L0X/api: VL53L0X_DEV
- VL53L0X/api_calibration: VL53L0X_WrByte()
- CMSIS/stm32f30x: DBGMCU_APB1PeriphConfig()
- drivers/paw3903_opticflow: paw3903_read_reg()
- API: X1Ranging, XRanging
- API: Motor, RxConfig
- CMSIS/cmFunc: __STATIC_INLINE
- CMSIS/cm0: __STATIC_INLINE
- CMSIS/cmInstr: __STATIC_INLINE
- drivers: serial_uart, serial
- StdPeriph/dac: DAC_TypeDef
- CMSIS + StdPeriph: gpio, rcc
- vcp/pwr: USB_Istr()
- drivers + StdPeriph: bus_spi, spi
- telemetry/hott: hottPrepareEAMResponse()
- CMSIS: sc000, cm0plus
- VL53L0X/api: VL53L0X_Error
- vcp/hw_config: usbVcpOpen()
- StdPeriph/exti: configureMPUDataReadyInterruptH…
- StdPeriph/hrtim: HRTIM_ADCTriggerConfig()
- StdPeriph/tim: ws2811LedStripHardwareInit()
- API + drivers: Serial-IO-Uart, opticflow_paw3903
- StdPeriph/opamp: OPAMP_InitTypeDef
- StdPeriph/adc: ADC_TypeDef
- sensors + common: gyro, maths
- API: Localisation, BMS
- drivers/bridge_sc18is602b: sc18_i2c_write()
- drivers/tof_manager_l1x: tofId_e
- drivers: pwm_output, tim
- drivers/light_ws2811strip_stm32f30x: ws2811DmaIsr()
- StdPeriph + config: flash, config
- drivers: compass_ak09916, opticflow_paw3903
- io/oled_display: Oled_DrawLine()
- io + drivers: display, printf
- VL53L1X/register_funcs: VL53L1_init_and_start_range()
- io/oled_display: sysText()
- flight + core: mw, FC-Control-Status
- API/Peripheral-GPIO: Peripheral_Init()
- StdPeriph/hrtim: HRTIM_OutputConfig()
- drivers/tof_manager: xRangingInit()
- common/typeconversion: tfp_format()
- drivers/bus_i2c_soft: I2C_delay()
- sensors/acceleration: rollAndPitchTrims_t
- drivers/display_ug2864hsweg01: i2c_OLED_send_cmd()
- sensors/acceleration: flightAccelCalData_T
- CMSIS/arm_math: float32_t
- StdPeriph/hrtim: HRTIM_TimingUnitBase_Config()
- user + API: PlutoPilot, mw
- VL53L1X/core: VL53L1_WrByte()
- StdPeriph/comp: COMP_InitTypeDef
- VL53L1X: api_core, core
- StdPeriph/tim: TIM_ICInit()
- flight/failsafe: useFailsafeConfig()
- common + flight: maths, imu
- CMSIS/arm_math: __QADD()
- CMSIS/arm_math: arm_pid_q15()
- VL53L0X/api: VL53L0X_PerformSingleRangingMea…
- API/RGB-LED: RGB_UpdateAnimation()
- flight/altitudehold: constrainf()
- CMSIS/arm_math: q15_t
- StdPeriph/hrtim: HRTIM_ExternalEventConfig()
- flight/mixer: mixerInit()
- StdPeriph/rcc: RCC_AHBPeriphResetCmd()
- API + blackbox: Debugging, blackbox
- flight + drivers: filter, pwm_output
- flight/imu: imuConfigure()
- common/maths: ring_avg_u16_t
- sensors + drivers: initialisation, accgyro_icm20948
- io + API: oled_display, Oled
- flight/mixer: feature()
- StdPeriph/rtc: ErrorStatus
- API/FC-Config-Setpoints: Failsafe_disable()
- drivers/light_led: void()
- CMSIS/arm_math: arm_bilinear_interp_q15()
- CMSIS/arm_math: arm_bilinear_interp_q31()
- rx: crsf, xbus
- CMSIS/arm_math: arm_bilinear_interp_q7()
- StdPeriph/i2c: I2C_TypeDef
- StdPeriph/usart: FunctionalState
- USB-FS: regs, sil
- CMSIS/cm3: __INLINE
- USB-FS/regs: Virtual_Com_Port_Reset()
- VL53L1X: ll_def, api_debug
- VL53L1X: api, api_core
- sensors/barometer: apmBaroCalibrate()
- flight/navigation: onGpsNewData()
- drivers: serial_1wire, system
- CMSIS/cm4_simd: __STATIC_INLINE
- StdPeriph: pwr, wwdg
- config/config: resetConf()
- config + flight: config, runtime_config
- io/serial_cli: cliPrint()
- flight/altitudehold: apmCalculateEstimatedAltitude()
- VL53L1X/register_funcs: VL53L1_ReadMulti()
- config + API: config, RxConfig
- flight/posEstimate: imuResetAccelerationSum()
- io/oled_display: Oled_FillRoundedRect()
- flight + io: pid, posControl
- telemetry: serial, frsky
- StdPeriph/hrtim: HRTIM_TypeDef
- flight/imu: drift_correction_yaw()
- flight/imu: imuUpdate()
- API/X1Ranging: laser_e
- API/RxConfig: Receiver_Aux_Config()
- common/atomic: __basepriSetMemRetVal()
- flight/posControl: resetPosIntegral()
- API/FC-Config-Setpoints: angle_e
- sensors + config: barometer, config
- flight/navigation: navigationInit()
- drivers + common: light_ws2811strip, colorconversion
- VL53L0X/api: VL53L0X_RdByte()
- CMSIS/rcc: RCC_GetFlagStatus()
- VL53L1X: wait, platform
- StdPeriph/can: CAN_TypeDef
- VL53L1X/def: ConvertModeFromLLD()
- StdPeriph/tim: TIM_TypeDef
- drivers/timer_stm32f30x: TIM_SelectOCxM_NoDisable()
- StdPeriph/spi: SPI_TypeDef
- VL53L0X/api_core: get_sequence_step_timeout()
- blackbox/blackbox: blackboxWrite()
- CMSIS/cm4: __STATIC_INLINE
- CMSIS/cm3: __STATIC_INLINE
- CMSIS/sc300: __STATIC_INLINE
- vcp: prop, int
- VL53L0X: i2c_platform, platform
- io/display: updateDisplay()
- StdPeriph/gpio: BitAction
- io/serial_1wire: escHardware_t
- CMSIS/gpio: BitAction

## God Nodes (most connected - your core abstractions)
1. `VL53L1_Error` - 284 edges
2. `VL53L1_DEV` - 202 edges
3. `FunctionalState` - 162 edges
4. `VL53L0X_Error` - 161 edges
5. `VL53L0X_DEV` - 144 edges
6. `TIM_TypeDef` - 121 edges
7. `HRTIM_TypeDef` - 77 edges
8. `millis()` - 73 edges
9. `init()` - 71 edges
10. `ADC_TypeDef` - 61 edges

## Surprising Connections (you probably didn't know these)
- `init()` --calls--> `plutoInit()`  [INFERRED]
  src/main/main.cpp → PlutoPilot.cpp
- `ws2811LedStripDMAEnable()` --calls--> `TIM_SetCounter()`  [INFERRED]
  src/main/drivers/light_ws2811strip_stm32f30x.c → lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_tim.c
- `serialICConfig()` --calls--> `TIM_ICInit()`  [INFERRED]
  src/main/drivers/serial_softserial.c → lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_tim.c
- `serialICConfig()` --calls--> `TIM_ICStructInit()`  [INFERRED]
  src/main/drivers/serial_softserial.c → lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_tim.c
- `vl53l0xConfigure()` --calls--> `VL53L0X_SetGpioConfig()`  [INFERRED]
  src/main/drivers/ranging_vl53l0x.cpp → lib/main/VL53L0X_API/core/src/vl53l0x_api.c

## Import Cycles
- None detected.

## Communities (251 total, 18 thin omitted)

### Community 105 - "USB-FS/core: DEVICE_INFO"
Cohesion: 0.05
Nodes (39): CONTROL_STATE, DEVICE, DEVICE_INFO, DEVICE_PROP, ENDPOINT_INFO, OneDescriptor, USER_STANDARD_REQUESTS, IN_DATA (+31 more)

### Community 115 - "drivers/sc18is602b: SC18IS602B"
Cohesion: 0.07
Nodes (23): SC18IS601B_GPIOPinMode, SC18IS601B_SPI_Mode, SC18IS601B_SPI_Speed, SC18IS602B, SC18IS601B_GPIO_MODE_INPUT_ONLY, SC18IS601B_GPIO_MODE_OPEN_DRAIN, SC18IS601B_GPIO_MODE_PUSH_PULL, SC18IS601B_GPIO_MODE_QUASI_BIDIRECTIONAL (+15 more)

### Community 12 - "io/serial: serialPortFunction_e"
Cohesion: 0.09
Nodes (31): serialConfig_s, serialPortConfig_s, serialPortUsage_s, doesConfigurationUsePort(), findNextSerialPortConfig(), findNextSharedSerialPort(), findSerialPortUsageByIdentifier(), findSerialPortUsageByPort() (+23 more)

### Community 120 - "drivers/serial: serialPort_s"
Cohesion: 0.12
Nodes (15): serialPort_s, baudRate, callback, identifier, mode, options, rxBuffer, rxBufferHead (+7 more)

### Community 121 - "flight/mixer: mixerMode"
Cohesion: 0.08
Nodes (26): mixerMode, MIXER_AIRPLANE, MIXER_ATAIL4, MIXER_BICOPTER, MIXER_CUSTOM, MIXER_CUSTOM_AIRPLANE, MIXER_CUSTOM_TRI, MIXER_DUALCOPTER (+18 more)

### Community 122 - "io/serial_msp: mspPort_s"
Cohesion: 0.10
Nodes (18): mspPort_s, systemReset(), systemResetToBootloader(), evaluateOtherData(), handleSerial(), mspProcess(), mspProcessReceivedData(), resetMspPort() (+10 more)

### Community 124 - "flight/altitudehold: AltTofL1x"
Cohesion: 0.11
Nodes (17): AltTofL1x, tofL1xDownClearNew(), tofL1xDownNew(), tofL1xDownOutOfRange(), tofL1xDownRangeMm(), ALT_TOF_DROPOUT_MS, ALT_TOF_HANDOVER_DOWN_CM, ALT_TOF_HANDOVER_UP_CM (+9 more)

### Community 136 - "flight/altitudehold: AltTofL0x"
Cohesion: 0.09
Nodes (16): AltTofL0x, tofDownClearNew(), tofDownFiltMm(), tofDownNew(), tofDownOutOfRange(), tofDownRawMm(), tofDownReseed(), ALT_TOF_DROPOUT_MS (+8 more)

### Community 146 - "io/rc_controls: adjustmentRange_s"
Cohesion: 0.08
Nodes (25): adjustmentRange_s, adjustmentState_s, modeActivationCondition_s, configureAdjustment(), isModeActivationConditionPresent(), isRangeActive(), updateActivatedModes(), updateAdjustmentStates() (+17 more)

### Community 150 - "USB-FS/core: RESULT"
Cohesion: 0.10
Nodes (26): RESULT, Data_Setup0(), In0_Process(), NoData_Setup0(), Out0_Process(), Post0_Process(), SetDeviceAddress(), Setup0_Process() (+18 more)

### Community 161 - "rx/rx: rxConfig_s"
Cohesion: 0.12
Nodes (16): rxConfig_s, channelRanges, failsafe_channel_configurations, maxcheck, midrc, mincheck, rcmap, rcSmoothing (+8 more)

### Community 167 - "drivers/barometer: baro_s"
Cohesion: 0.15
Nodes (16): baro_s, calculate, get_up, get_ut, measurment_start, read, start_up, start_ut (+8 more)

### Community 173 - "StdPeriph/crc: IRQn"
Cohesion: 0.08
Nodes (10): IRQn, CRC_ReverseOutputDataCmd(), BusFault_IRQn, DebugMonitor_IRQn, MemoryManagement_IRQn, NonMaskableInt_IRQn, PendSV_IRQn, SVCall_IRQn (+2 more)

### Community 176 - "blackbox/blackbox_fielddefs: flightLogEventData_u"
Cohesion: 0.16
Nodes (15): flightLogEvent_syncBeep_s, flightLogEventData_u, blackboxConditionalFieldDefinition_t, blackboxDeltaFieldDefinition_t, blackboxSimpleFieldDefinition_t, flightLogEvent_gtuneCycleResult_t, flightLogEvent_inflightAdjustment_t, flightLogEvent_loggingResume_t (+7 more)

### Community 177 - "flight/failsafe: failsafeState_s"
Cohesion: 0.12
Nodes (16): failsafeState_s, failsafePhase(), failsafePhase_e, failsafeRxLinkState_e, active, events, landingShouldBeFinishedAt, monitoring (+8 more)

### Community 178 - "flight/pid: pidProfile_s"
Cohesion: 0.12
Nodes (16): pidProfile_s, A_level, D8, D_f, dterm_cut_hz, gyro_cut_hz, H_level, H_sensitivity (+8 more)

### Community 18 - "drivers/timer: timerHardware_t"
Cohesion: 0.08
Nodes (46): timerCCHandlerRec_s, timerOvrHandlerRec_s, TIM_ClearFlag(), TIM_ICStructInit(), TIM_ITConfig(), ppmAvoidPWMTimerClash(), ppmInConfig(), ppmInit() (+38 more)

### Community 188 - "drivers/pwm_mapping: drv_pwm_config_s"
Cohesion: 0.13
Nodes (15): drv_pwm_config_s, airplane, idlePulse, motorPwmRate, sonarGPIOConfig, useCurrentMeterADC, useLEDStrip, useOneshot (+7 more)

### Community 189 - "blackbox/blackbox: blackboxSlowState_s"
Cohesion: 0.20
Nodes (10): blackboxSlowState_s, loadSlowState(), writeSlowFrameIfNeeded(), rxAreFlightChannelsValid(), failsafePhase, flightModeFlags, rxFlightChannelsValid, rxSignalReceived (+2 more)

### Community 192 - "blackbox/blackbox: blackboxMainState_s"
Cohesion: 0.14
Nodes (14): blackboxMainState_s, accSmooth, amperageLatest, axisPID_D, axisPID_I, axisPID_P, gyroADC, logVar (+6 more)

### Community 193 - "flight/lowpass: lowpass_s"
Cohesion: 0.14
Nodes (14): lowpass_s, a, af, b, bf, coeff_shift, freq, init (+6 more)

### Community 2 - "drivers + API: serial, system"
Cohesion: 0.05
Nodes (36): serialPortVTable, findSerialPortConfigState_s, findSharedSerialPortState_s, debugPrint(), Monitor_Print(), Monitor_Println(), serializeString(), command_add() (+28 more)

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
Cohesion: 0.15
Nodes (12): oaCoreState, oaCoreDistance(), oaCoreMap(), oaCoreRun(), filtMm, lastPitchOA, lastRollOA, lastRunMs (+4 more)

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

### Community 245 - "API/Motor: bidirectional_motor"
Cohesion: 0.29
Nodes (7): bidirectional_motor, M1, M2, M5, M6, M7, M8

### Community 246 - "API/Scheduler-Timer: Interval"
Cohesion: 0.29
Nodes (4): Interval, loopTime, repeat, time

### Community 247 - "API/XRanging: laser_sensors"
Cohesion: 0.29
Nodes (7): laser_sensors, BACK, DOWN, EXTERNAL, FRONT, LEFT, RIGHT

### Community 248 - "blackbox/blackbox_fielddefs: blackboxConditionalFieldDefinit…"
Cohesion: 0.29
Nodes (7): blackboxConditionalFieldDefinition_s, condition, encode, fieldNameIndex, isSigned, name, predict

### Community 249 - "flight/imu: imuRuntimeConfig_s"
Cohesion: 0.29
Nodes (7): imuRuntimeConfig_s, acc_lpf_factor, acc_unarmedcal, gyro_cmpf_factor, gyro_cmpfm_factor, gyro_lpf_factor, small_angle

### Community 250 - "io/gps: gpsDataNmea_s"
Cohesion: 0.10
Nodes (19): gpsDataNmea_s, gpsInitData_s, GPS_coord_to_degrees(), gpsNewFrame(), gpsNewFrameNMEA(), gpsNewFrameUBLOX(), grab_fields(), shiftPacketLog() (+11 more)

### Community 255 - "API/XRanging: laser_status"
Cohesion: 0.33
Nodes (6): laser_status, LASER_BOOT_FAILED, LASER_I2C_ERROR, LASER_NOT_ENABLED, LASER_OK, LASER_OUT_OF_RANGE

### Community 256 - "blackbox/blackbox_fielddefs: blackboxSimpleFieldDefinition_s"
Cohesion: 0.33
Nodes (6): blackboxSimpleFieldDefinition_s, encode, fieldNameIndex, isSigned, name, predict

### Community 257 - "common/maths: stdev_s"
Cohesion: 0.33
Nodes (6): stdev_s, m_n, m_newM, m_newS, m_oldM, m_oldS

### Community 258 - "drivers/sound_beeper: beeperConfig_s"
Cohesion: 0.33
Nodes (6): beeperConfig_s, gpioMode, gpioPeripheral, gpioPin, gpioPort, isInverted

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

### Community 266 - "drivers/tof_dev_area: tofDevArea_u"
Cohesion: 0.50
Nodes (3): tofDevArea_u, l0x, l1x

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

### Community 274 - "io/rc_controls: adjustmentConfig_u"
Cohesion: 0.40
Nodes (5): adjustmentConfig_u, selectConfig, stepConfig, adjustmentSelectConfig_t, adjustmentStepConfig_t

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

### Community 3 - "flight + io: pid, rc_controls"
Cohesion: 0.10
Nodes (36): escAndServoConfig_s, gimbalConfig_s, rxConfig_s, pidSetController(), useGyroConfig(), blackboxFieldDefinition_t, blackboxGpsState_t, blackboxMainState_t (+28 more)

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

### Community 312 - "drivers/pwm_mapping: pwmOutputConfiguration_s"
Cohesion: 0.67
Nodes (3): pwmOutputConfiguration_s, motorCount, servoCount

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

### Community 37 - "drivers + StdPeriph: dma, serial"
Cohesion: 0.08
Nodes (40): portOptions_t, DMA_ClearITPendingBit(), DMA_Cmd(), DMA_DeInit(), DMA_GetCurrDataCounter(), DMA_Init(), DMA_ITConfig(), DMA_SetCurrDataCounter() (+32 more)

### Community 44 - "API/Peripheral-ADC: APIAdcInit()"
Cohesion: 0.20
Nodes (12): ADC_PinMapping, _Adc1init(), _Adc2init(), _Adc3init(), _Adc4init(), APIAdcInit(), Peripheral_Init(), Peripheral_Read() (+4 more)

### Community 51 - "drivers/timer: timerCCHandlerRec_t"
Cohesion: 0.21
Nodes (12): timerConfig_s, ppmEdgeCallback(), ppmOverflowCallback(), pwmEdgeCallback(), pwmOverflowCallback(), captureCompare_t, timerCCHandlerRec_t, edgeCallback (+4 more)

### Community 55 - "io + rx: gps, rx"
Cohesion: 0.04
Nodes (39): gpsConfig_s, ubloxSbas_s, rxFailsafeChannelConfiguration_s, rxRuntimeConfig_s, hottSendEAMResponse(), hottSendGPSResponse(), hottSendResponse(), processBinaryModeRequest() (+31 more)

### Community 68 - "API/Motor: Rev_Motor_Gpio"
Cohesion: 0.50
Nodes (4): Rev_Motor_Gpio, gpio, pin, RCC_AHBPeriph

### Community 69 - "drivers/pwm_mapping: sonarGPIOConfig_s"
Cohesion: 0.50
Nodes (4): sonarGPIOConfig_s, echoPin, gpio, triggerPin

### Community 70 - "flight/motor: Rev_Motor_Gpio"
Cohesion: 0.50
Nodes (4): Rev_Motor_Gpio, gpio, pin, RCC_AHBPeriph

### Community 74 - "API + command: Scheduler-Timer, command"
Cohesion: 0.07
Nodes (41): ScheduledFn, Command_Arm(), ensure_task_init(), Execute_Scheduled(), Interval::check(), Scheduler_Add(), Scheduler_Cancel(), Scheduler_Clear() (+33 more)

### Community 75 - "drivers/accgyro_mpu: lpf_e"
Cohesion: 0.06
Nodes (39): accel_fsr_e, clock_sel_e, gyro_fsr_e, lpf_e, mpuConfiguration_s, mpuDetectionResult_s, INV_FSR_16G, INV_FSR_2G (+31 more)

### Community 76 - "API/Motor: motor_direction"
Cohesion: 0.67
Nodes (3): motor_direction, ANTICLOCK_WISE, CLOCK_WISE

### Community 79 - "io/gps: gpsCoordinateDDDMMmmmm_s"
Cohesion: 0.67
Nodes (3): gpsCoordinateDDDMMmmmm_s, dddmm, mmmm

### Community 88 - "drivers/pwm_rx: ppmDevice_s"
Cohesion: 0.18
Nodes (11): ppmDevice_s, captures, currentTime, deltaTime, largeCounter, numChannels, numChannelsPrevFrame, previousTime (+3 more)

### Community 90 - "API/XRanging: LaserMap"
Cohesion: 0.11
Nodes (26): GpioAlias, LaserMap, adcPinEnabled(), Peripheral_Init(), x1RangingAltLatch(), x1RangingAltXshut(), altPinConflict(), altRequest() (+18 more)

### Community 0 - "io + flight: beeper, failsafe"
Cohesion: 0.07
Nodes (42): Command_DisArm(), applyObjectAvoidance(), cmp32(), reset_FSI(), set_FSI(), status_FSI(), limitAltitude(), failsafeApplyControlInput() (+34 more)

### Community 1 - "rx: rx, gps"
Cohesion: 0.16
Nodes (28): blackboxDeviceOpen(), featureClear(), gpsInit(), findSerialPortConfig(), openSerialPort(), crsfInit(), crsfReadRawRC(), rxMspInit() (+20 more)

### Community 10 - "telemetry: frsky, hott"
Cohesion: 0.16
Nodes (17): closeSerialPort(), mspReleasePortIfAllocated(), mspSetTelemetryPort(), checkFrSkyTelemetryState(), configureFrSkyTelemetryPort(), freeFrSkyTelemetryPort(), checkHoTTTelemetryState(), configureHoTTTelemetryPort() (+9 more)

### Community 100 - "sensors/battery: BMS_Update()"
Cohesion: 0.11
Nodes (32): INA219_Config(), INA219_Init(), INA219_ReadBus_mV(), INA219_ReadShunt_10uV(), INA219_RegWrite(), ina219ReadReg(), addSatU32(), batteryCountAvailable() (+24 more)

### Community 102 - "rx/rx: detectAndApplySignalLossBehavio…"
Cohesion: 0.11
Nodes (24): adcGetChannel(), isPPMDataBeingReceived(), isPWMDataBeingReceived(), resetPPMDataReceivedState(), rxMspFrameComplete(), applyRxChannelRangeConfiguraton(), calculateChannelRemapping(), calculateNonDataDrivenChannel() (+16 more)

### Community 103 - "io: flashfs, serial_msp"
Cohesion: 0.06
Nodes (69): Set_LED(), blackboxDeviceReserveBufferSpace(), FC_Reboot_Led(), m25p16_eraseCompletely(), m25p16_eraseSector(), m25p16_getGeometry(), m25p16_isReady(), m25p16_pageProgram() (+61 more)

### Community 104 - "CMSIS/arm_math: __INLINE"
Cohesion: 0.14
Nodes (28): arm_circularRead_f32(), arm_circularRead_q7(), arm_circularWrite_f32(), arm_circularWrite_q7(), arm_linear_interp_q31(), arm_linear_interp_q7(), clip_q31_to_q15(), clip_q31_to_q7() (+20 more)

### Community 106 - "StdPeriph/flash: FLASH_WaitForLastOperation()"
Cohesion: 0.15
Nodes (19): FLASH_EraseAllPages(), FLASH_GetFlagStatus(), FLASH_GetStatus(), FLASH_HalfCycleAccessCmd(), FLASH_ITConfig(), FLASH_OB_BOOTConfig(), FLASH_OB_EnableWRP(), FLASH_OB_Erase() (+11 more)

### Community 109 - "telemetry/frsky: handleFrSkyTelemetry()"
Cohesion: 0.23
Nodes (26): GPStoDDDMM_MMMM(), handleFrSkyTelemetry(), hasEnoughTimeLapsedSinceLastTelemetryTransmission(), sendAccel(), sendAmperage(), sendBaro(), sendDataHead(), sendFakeLatLong() (+18 more)

### Community 11 - "API + flight: altitudehold, FC-Config-Setpoints"
Cohesion: 0.12
Nodes (14): DesiredPosition_setRelative(), getDesiredPositions(), setDesiredPosition(), Estimate_Get(), Sensor_Get(), Localisation_P::get(), setTakeOffGoal(), altHoldReleaseGroundIdle() (+6 more)

### Community 110 - "io/ledstrip: updateLedStrip()"
Cohesion: 0.10
Nodes (32): scaleRange(), getLedHsv(), isWS2811LedStripReady(), scaleLedValue(), setLedHsv(), setStripColor(), setStripColors(), applyDefaultColors() (+24 more)

### Community 112 - "drivers/serial_softserial: openSoftSerial()"
Cohesion: 0.14
Nodes (25): TIM_SetCounter(), applyChangedBits(), extractAndStoreRxByte(), isSoftSerialTransmitBufferEmpty(), onSerialRxPinChange(), onSerialTimer(), openSoftSerial(), prepareForNextRxByte() (+17 more)

### Community 116 - "telemetry + io: hott, smartport"
Cohesion: 0.10
Nodes (26): sensorsClear(), serialRead(), serialRxBytesWaiting(), serialSetMode(), gpsEnablePassthrough(), gpsNewData(), gpsThread(), cliGpsPassthrough() (+18 more)

### Community 117 - "API/FC-Control-Command: Command_Flip()"
Cohesion: 0.50
Nodes (3): Command_Flip(), Command_TakeOff(), landingInProgress()

### Community 118 - "VL53L1X/api_preset_modes: VL53L1_set_preset_mode()"
Cohesion: 0.39
Nodes (18): VL53L1_set_preset_mode(), VL53L1_init_tuning_parm_storage_struct(), VL53L1_preset_mode_low_power_auto_long_ranging(), VL53L1_preset_mode_low_power_auto_ranging(), VL53L1_preset_mode_low_power_auto_short_ranging(), VL53L1_preset_mode_olt(), VL53L1_preset_mode_singleshot_ranging(), VL53L1_preset_mode_standard_ranging() (+10 more)

### Community 127 - "VL53L0X/api: VL53L0X_DEV"
Cohesion: 0.11
Nodes (47): VL53L0X_apply_offset_adjustment(), VL53L0X_get_offset_calibration_data_micro_meter(), VL53L0X_perform_offset_calibration(), VL53L0X_perform_xtalk_calibration(), VL53L0X_set_offset_calibration_data_micro_meter(), VL53L0X_get_pal_range_status(), VL53L0X_get_total_signal_rate(), VL53L0X_get_total_xtalk_rate() (+39 more)

### Community 128 - "VL53L0X/api_calibration: VL53L0X_WrByte()"
Cohesion: 0.26
Nodes (19): count_enabled_spads(), enable_ref_spads(), enable_spad_bit(), get_next_good_spad(), get_ref_spad_map(), is_aperture(), perform_ref_signal_measurement(), set_ref_spad_map() (+11 more)

### Community 13 - "CMSIS/stm32f30x: DBGMCU_APB1PeriphConfig()"
Cohesion: 0.04
Nodes (40): DBGMCU_APB1PeriphConfig(), DBGMCU_APB2PeriphConfig(), DBGMCU_Config(), IWDG_GetFlagStatus(), ADC_Common_TypeDef, CAN_FIFOMailBox_TypeDef, CAN_FilterRegister_TypeDef, CAN_TxMailBox_TypeDef (+32 more)

### Community 130 - "drivers/paw3903_opticflow: paw3903_read_reg()"
Cohesion: 0.15
Nodes (19): paw3903_check_id(), paw3903_get_mode(), paw3903_get_orientation(), paw3903_get_resolution(), paw3903_init(), paw3903_power_up_reset(), paw3903_read_motion(), paw3903_read_motion_burst() (+11 more)

### Community 132 - "API: X1Ranging, XRanging"
Cohesion: 0.25
Nodes (8): xshutPinInit(), x1RangingInit(), x1RangingRequested(), rangingFamilyLatch(), xRangingSideXshut(), m25p16_release(), tofL1xReset(), tofL1xStartSides()

### Community 135 - "API: Motor, RxConfig"
Cohesion: 0.22
Nodes (14): Motor_Init(), Motor_Set(), Motor_SetDir(), xshutPinWrite(), ESP_WiFi_OFF(), ESP_WiFi_ON(), STM_PB3_ESP_IO14_Init(), digitalHi() (+6 more)

### Community 137 - "CMSIS/cmFunc: __STATIC_INLINE"
Cohesion: 0.17
Nodes (17): __get_APSR(), __get_BASEPRI(), __get_CONTROL(), __get_FAULTMASK(), __get_FPSCR(), __get_IPSR(), __get_MSP(), __get_PRIMASK() (+9 more)

### Community 138 - "CMSIS/cm0: __STATIC_INLINE"
Cohesion: 0.12
Nodes (22): NVIC_ClearPendingIRQ(), NVIC_DisableIRQ(), NVIC_EnableIRQ(), NVIC_GetPendingIRQ(), NVIC_GetPriority(), NVIC_SetPendingIRQ(), NVIC_SetPriority(), NVIC_SystemReset() (+14 more)

### Community 139 - "CMSIS/cmInstr: __STATIC_INLINE"
Cohesion: 0.18
Nodes (19): __CLREX(), __CLZ(), __DMB(), __ISB(), __LDREXB(), __LDREXH(), __LDREXW(), __NOP() (+11 more)

### Community 14 - "drivers: serial_uart, serial"
Cohesion: 0.10
Nodes (27): fputc(), _putc(), setPrintfSerialPort(), tfp_printf(), isSerialTransmitBufferEmpty(), serialGetBaudRate(), serialPrint(), serialSetBaudRate() (+19 more)

### Community 140 - "StdPeriph/dac: DAC_TypeDef"
Cohesion: 0.18
Nodes (19): DAC_ClearFlag(), DAC_ClearITPendingBit(), DAC_Cmd(), DAC_DeInit(), DAC_DMACmd(), DAC_DualSoftwareTriggerCmd(), DAC_GetDataOutputValue(), DAC_GetFlagStatus() (+11 more)

### Community 143 - "CMSIS + StdPeriph: gpio, rcc"
Cohesion: 0.09
Nodes (30): GPIO_DeInit(), GPIO_Init(), GPIO_PinAFConfig(), GPIO_PinLockConfig(), GPIO_ReadInputData(), GPIO_ReadInputDataBit(), GPIO_ReadOutputData(), GPIO_ReadOutputDataBit() (+22 more)

### Community 144 - "vcp/pwr: USB_Istr()"
Cohesion: 0.24
Nodes (9): Leave_LowPowerMode(), USB_Cable_Config(), USB_LP_CAN1_RX0_IRQHandler(), USB_Istr(), PowerOff(), PowerOn(), Resume(), Resume_Init() (+1 more)

### Community 147 - "drivers + StdPeriph: bus_spi, spi"
Cohesion: 0.15
Nodes (24): SPI_Cmd(), SPI_I2S_DeInit(), SPI_I2S_GetFlagStatus(), SPI_Init(), SPI_ReceiveData8(), SPI_RxFIFOThresholdConfig(), SPI_SendData8(), getPrescaler() (+16 more)

### Community 148 - "telemetry/hott: hottPrepareEAMResponse()"
Cohesion: 0.21
Nodes (12): addGPSCoordinates(), hottEAMUpdateBattery(), hottEAMUpdateBatteryDrawnCapacity(), hottEAMUpdateCurrentMeter(), hottPrepareEAMResponse(), hottPrepareGPSResponse(), hottPrepareMessages(), initialiseEAMMessage() (+4 more)

### Community 149 - "CMSIS: sc000, cm0plus"
Cohesion: 0.08
Nodes (37): NVIC_ClearPendingIRQ(), NVIC_DisableIRQ(), NVIC_EnableIRQ(), NVIC_GetPendingIRQ(), NVIC_GetPriority(), NVIC_SetPendingIRQ(), NVIC_SetPriority(), SysTick_Config() (+29 more)

### Community 15 - "VL53L0X/api: VL53L0X_Error"
Cohesion: 0.07
Nodes (56): VL53L0X_get_device_error_string(), VL53L0X_get_limit_check_info(), VL53L0X_get_pal_error_string(), VL53L0X_get_pal_state_string(), VL53L0X_get_range_status_string(), VL53L0X_get_sequence_steps_info(), VL53L0X_CheckAndLoadInterruptSettings(), VL53L0X_EnableInterruptMask() (+48 more)

### Community 151 - "vcp/hw_config: usbVcpOpen()"
Cohesion: 0.19
Nodes (11): USB_Init(), SetEPTxValid(), usbVcpOpen(), usbVcpWrite(), CDC_Send_DATA(), Get_SerialNum(), IntToUnicode(), Set_System() (+3 more)

### Community 152 - "StdPeriph/exti: configureMPUDataReadyInterruptH…"
Cohesion: 0.14
Nodes (13): EXTI_ClearITPendingBit(), EXTI_GetFlagStatus(), EXTI_GetITStatus(), EXTI_Init(), EXTI_StructInit(), SYSCFG_EXTILineConfig(), configureMPUDataReadyInterruptHandling(), MPU_DATA_READY_EXTI_Handler() (+5 more)

### Community 153 - "StdPeriph/hrtim: HRTIM_ADCTriggerConfig()"
Cohesion: 0.11
Nodes (18): HRTIM_ADCTriggerConfig(), HRTIM_BurstModeConfig(), HRTIM_ChopperModeConfig(), HRTIM_DeadTimeConfig(), HRTIM_FaultConfig(), HRTIM_SynchronizationConfig(), HRTIM_TimerEventFilteringConfig(), HRTIM_WaveformCaptureConfig() (+10 more)

### Community 154 - "StdPeriph/tim: ws2811LedStripHardwareInit()"
Cohesion: 0.29
Nodes (16): TIM_DMACmd(), TIM_OC1Init(), TIM_OC1PreloadConfig(), TIM_OC2Init(), TIM_OC2PreloadConfig(), TIM_OC3Init(), TIM_OC3PreloadConfig(), TIM_OC4Init() (+8 more)

### Community 158 - "API + drivers: Serial-IO-Uart, opticflow_paw3903"
Cohesion: 0.16
Nodes (16): getBaud(), Uart_Init(), Uart_Read16(), Uart_Read32(), Uart_Read8(), Uart_rxBytesWaiting(), Uart_txBytesFree(), Uart_Write() (+8 more)

### Community 159 - "StdPeriph/opamp: OPAMP_InitTypeDef"
Cohesion: 0.13
Nodes (9): OPAMP_Cmd(), OPAMP_Init(), OPAMP_StartCalibration(), OPAMP_StructInit(), OPAMP_TimerControlledMuxCmd(), OPAMP_TimerControlledMuxConfig(), OPAMP_VrefConnectADCCmd(), OPAMP_VrefConnectNonInvertingInput() (+1 more)

### Community 16 - "StdPeriph/adc: ADC_TypeDef"
Cohesion: 0.06
Nodes (71): ADC_AnalogWatchdog1SingleChannelConfig(), ADC_AnalogWatchdog1ThresholdsConfig(), ADC_AnalogWatchdog2SingleChannelConfig(), ADC_AnalogWatchdog2ThresholdsConfig(), ADC_AnalogWatchdog3SingleChannelConfig(), ADC_AnalogWatchdog3ThresholdsConfig(), ADC_AnalogWatchdogCmd(), ADC_AutoDelayCmd() (+63 more)

### Community 162 - "sensors + common: gyro, maths"
Cohesion: 0.21
Nodes (12): devClear(), devPush(), devStandardDeviation(), devVariance(), applyGyroZero(), gyroSetCalibrationCycles(), gyroUpdate(), isGyroCalibrationComplete() (+4 more)

### Community 163 - "API: Localisation, BMS"
Cohesion: 0.09
Nodes (4): Bms_Get(), Localisation_P::init(), localisation_type, oled_mode_e

### Community 168 - "drivers/bridge_sc18is602b: sc18_i2c_write()"
Cohesion: 0.22
Nodes (12): sc18_address_cfg(), sc18_clearInterrupt(), sc18_configureSPI(), sc18_enableGPIO(), sc18_i2c_read(), sc18_i2c_write(), sc18_readGPIO(), sc18_setLowPowerMode() (+4 more)

### Community 169 - "drivers/tof_manager_l1x: tofId_e"
Cohesion: 0.26
Nodes (16): l1xBe16(), l1xDecodeResult(), l1xRangeStatus(), tofIdValid(), tofL1xBusError(), tofL1xCheckStale(), tofL1xEntry(), tofL1xLastSampleMs() (+8 more)

### Community 17 - "drivers: pwm_output, tim"
Cohesion: 0.15
Nodes (17): TIM_Cmd(), TIM_CtrlPWMOutputs(), TIM_TimeBaseInit(), TIM_TimeBaseStructInit(), ws2811LedStripDMAEnable(), pwmInit(), isMotorBrushed(), pwmBrushedMotorConfig() (+9 more)

### Community 170 - "drivers/light_ws2811strip_stm32f30x: ws2811DmaIsr()"
Cohesion: 0.26
Nodes (10): DMA_ClearFlag(), DMA_GetFlagStatus(), DMA1_Channel1_IRQHandler(), DMA1_Channel2_IRQHandler(), DMA1_Channel5_IRQHandler(), DMA1_Channel6_IRQHandler(), DMA2_Channel3_IRQHandler(), DMA2_Channel5_IRQHandler() (+2 more)

### Community 171 - "StdPeriph + config: flash, config"
Cohesion: 0.24
Nodes (10): FLASH_ClearFlag(), FLASH_ErasePage(), FLASH_Lock(), FLASH_ProgramWord(), FLASH_Unlock(), calculateChecksum(), ensureEEPROMContainsValidData(), isEEPROMContentValid() (+2 more)

### Community 172 - "drivers: compass_ak09916, opticflow_paw3903"
Cohesion: 0.13
Nodes (22): SPI_Write(), sc18_reset(), ak09916Detect(), ak09916Init(), ak09916Read(), ak09916SensorMagRead(), ak09916SensorMagWrite(), ak09916SensorRead() (+14 more)

### Community 174 - "io/oled_display: Oled_DrawLine()"
Cohesion: 0.18
Nodes (11): Oled_DeadEyes(), Oled_Line(), Oled_PitchLine(), Oled_RollLine(), Oled_DrawArrow(), Oled_DrawBoxyXEye(), Oled_DrawLine(), Oled_DrawPitchIndicator() (+3 more)

### Community 180 - "io + drivers: display, printf"
Cohesion: 0.08
Nodes (31): resetUser(), init_printf(), printfSupportInit(), initEEPROM(), latchActiveFeatures(), sensors(), m25p16_init(), m25p16_readIdentification() (+23 more)

### Community 183 - "VL53L1X/register_funcs: VL53L1_init_and_start_range()"
Cohesion: 0.16
Nodes (12): VL53L1_init_and_start_range(), VL53L1_i2c_decode_int16_t(), VL53L1_i2c_encode_int16_t(), VL53L1_update_ll_driver_cfg_state(), VL53L1_update_ll_driver_rd_state(), VL53L1_get_customer_nvm_managed(), VL53L1_get_system_control(), VL53L1_i2c_decode_customer_nvm_managed() (+4 more)

### Community 186 - "io/oled_display: sysText()"
Cohesion: 0.17
Nodes (22): Oled_Is_Mode(), Oled_Mode(), dispArmData(), dispDevMode(), dispFlightStatus(), dispRxConnection(), dispStopWatch(), dispVoltage() (+14 more)

### Community 19 - "flight + core: mw, FC-Control-Status"
Cohesion: 0.12
Nodes (16): FlightStatus_Check(), FlightStatus_Get(), leastSignificantBit(), flightStatusIndicator(), init_Gtune(), updateDelayCycles(), calculateThrottleAngleCorrection(), MotroWakeUp() (+8 more)

### Community 194 - "API/Peripheral-GPIO: Peripheral_Init()"
Cohesion: 0.29
Nodes (4): Peripheral_Init(), Peripheral_Read(), Peripheral_Write(), digitalIn()

### Community 195 - "StdPeriph/hrtim: HRTIM_OutputConfig()"
Cohesion: 0.19
Nodes (13): HRTIM_CompareUnitConfig(), HRTIM_OutputConfig(), HRTIM_SimpleOCChannelConfig(), HRTIM_SimpleOnePulseChannelConfig(), HRTIM_SimplePWMChannelConfig(), HRTIM_TIM_ResetConfig(), HRTIM_WaveformCompareConfig(), HRTIM_WaveformOutputConfig() (+5 more)

### Community 197 - "drivers/tof_manager: xRangingInit()"
Cohesion: 0.19
Nodes (16): rangingFamily(), xRangingInit(), updateSpiOpticFlow(), vl53l0xProbe(), tofIdValid(), tofManagerBringUp(), tofManagerEntry(), tofManagerMarkFailed() (+8 more)

### Community 198 - "common/typeconversion: tfp_format()"
Cohesion: 0.27
Nodes (9): putchw(), tfp_format(), a2d(), a2i(), i2a(), li2a(), ui2a(), uli2a() (+1 more)

### Community 199 - "drivers/bus_i2c_soft: I2C_delay()"
Cohesion: 0.44
Nodes (11): I2C_Ack(), I2C_delay(), I2C_NoAck(), I2C_ReceiveByte(), I2C_SendByte(), I2C_Start(), I2C_Stop(), I2C_WaitAck() (+3 more)

### Community 20 - "sensors/acceleration: rollAndPitchTrims_t"
Cohesion: 0.14
Nodes (21): readEEPROMAndNotify(), resetAccelerometerTrims(), saveConfigAndNotify(), applyAndSaveAccelerometerTrimsDelta(), applyAccelerationTrims(), calibrate_accel(), calibrate_find_delta(), calibrate_reset_matrices() (+13 more)

### Community 200 - "drivers/display_ug2864hsweg01: i2c_OLED_send_cmd()"
Cohesion: 0.32
Nodes (13): bitreverse8(), i2c_OLED_clear_display(), i2c_OLED_clear_display_quick(), i2c_OLED_draw_bitmap(), i2c_OLED_draw_rect(), i2c_OLED_send_byte(), i2c_OLED_send_bytes(), i2c_OLED_send_changed_bytes() (+5 more)

### Community 202 - "sensors/acceleration: flightAccelCalData_T"
Cohesion: 0.50
Nodes (4): resetAccelerometerCalibration(), getAccelerationCalibration(), setAccelerationCalibration(), flightAccelCalData_T

### Community 203 - "CMSIS/arm_math: float32_t"
Cohesion: 0.17
Nodes (12): arm_bilinear_interp_f32(), arm_clarke_f32(), arm_inv_clarke_f32(), arm_inv_park_f32(), arm_linear_interp_f32(), arm_park_f32(), arm_pid_f32(), arm_sqrt_f32() (+4 more)

### Community 204 - "StdPeriph/hrtim: HRTIM_TimingUnitBase_Config()"
Cohesion: 0.32
Nodes (12): HRTIM_MasterBase_Config(), HRTIM_MasterWaveform_Config(), HRTIM_SimpleBase_Init(), HRTIM_SimpleCapture_Init(), HRTIM_SimpleOC_Init(), HRTIM_SimpleOnePulse_Init(), HRTIM_SimplePWM_Init(), HRTIM_TimingUnitBase_Config() (+4 more)

### Community 206 - "user + API: PlutoPilot, mw"
Cohesion: 0.18
Nodes (11): onLoopFinish(), onLoopStart(), plutoInit(), plutoLoop(), resetUserRCflag(), userRCrelease(), FlightMode_Check(), FlightMode_Set() (+3 more)

### Community 21 - "VL53L1X/core: VL53L1_WrByte()"
Cohesion: 0.05
Nodes (48): VL53L1_run_device_test(), VL53L1_run_ref_spad_char(), VL53L1_run_spad_rate_map(), VL53L1_set_GPIO_interrupt_config(), VL53L1_init_ssc_config_struct(), VL53L1_nvm_enable(), VL53L1_calc_pll_period_us(), VL53L1_decode_row_col() (+40 more)

### Community 213 - "StdPeriph/comp: COMP_InitTypeDef"
Cohesion: 0.22
Nodes (6): COMP_Cmd(), COMP_Init(), COMP_StructInit(), COMP_SwitchCmd(), COMP_WindowCmd(), COMP_InitTypeDef

### Community 215 - "VL53L1X: api_core, core"
Cohesion: 0.24
Nodes (8): VL53L1_copy_sys_and_core_results_to_range_results(), VL53L1_get_device_results(), VL53L1_get_measurement_results(), VL53L1_print_range_results(), VL53L1_check_ll_driver_rd_state(), VL53L1_low_power_auto_setup_manual_calibration(), VL53L1_low_power_auto_update_DSS(), VL53L1_range_results_t

### Community 22 - "StdPeriph/tim: TIM_ICInit()"
Cohesion: 0.17
Nodes (15): TI1_Config(), TI2_Config(), TI3_Config(), TI4_Config(), TIM_BDTRConfig(), TIM_BDTRStructInit(), TIM_ICInit(), TIM_PWMIConfig() (+7 more)

### Community 222 - "flight/failsafe: useFailsafeConfig()"
Cohesion: 0.67
Nodes (3): failsafeReset(), useFailsafeConfig(), failsafeConfig_t

### Community 23 - "common + flight: maths, imu"
Cohesion: 0.20
Nodes (14): buildRotationMatrix(), cos_approx(), degreesToRadians(), normalizeV(), rotateV(), sin_approx(), imuCalculateEstimatedAttitude(), imuCalculateHeading() (+6 more)

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

### Community 25 - "flight/altitudehold: constrainf()"
Cohesion: 0.10
Nodes (24): setUserLoopFrequency(), constrainf(), altLaserFusionLatch(), applyAltHold(), applyFixedWingAltHold(), applyMultirotorAltHold(), calculateAltHoldThrottleAdjustment(), calculateTiltAngle() (+16 more)

### Community 251 - "CMSIS/arm_math: q15_t"
Cohesion: 0.33
Nodes (6): arm_circularRead_q15(), arm_circularWrite_q15(), arm_linear_interp_q15(), arm_recip_q15(), __CLZ(), q15_t

### Community 252 - "StdPeriph/hrtim: HRTIM_ExternalEventConfig()"
Cohesion: 0.40
Nodes (6): HRTIM_CaptureUnitConfig(), HRTIM_EventConfig(), HRTIM_ExternalEventConfig(), HRTIM_SimpleCaptureChannelConfig(), HRTIM_BasicCaptureChannelCfgTypeDef, HRTIM_EventCfgTypeDef

### Community 254 - "flight/mixer: mixerInit()"
Cohesion: 0.33
Nodes (3): mixerInit(), mixerLoadMix(), servoMixerLoadMix()

### Community 26 - "StdPeriph/rcc: RCC_AHBPeriphResetCmd()"
Cohesion: 0.05
Nodes (14): RCC_AHBPeriphClockCmd(), RCC_AHBPeriphResetCmd(), RCC_APB1PeriphClockCmd(), RCC_APB2PeriphClockCmd(), RCC_BackupResetCmd(), RCC_ClockSecuritySystemCmd(), RCC_GetClocksFreq(), RCC_GetFlagStatus() (+6 more)

### Community 267 - "flight + drivers: filter, pwm_output"
Cohesion: 0.12
Nodes (12): biquadApply(), biquadInitLPF(), initIMUFilters(), generateLowpassCoeffs2(), lowpassFixed(), filterServos(), pwmWriteFuncPtr, timCCER_t (+4 more)

### Community 268 - "flight/imu: imuConfigure()"
Cohesion: 0.38
Nodes (5): calculateAccZLowPassFilterRCTimeConstant(), calculateThrottleAngleScale(), imuConfigure(), accDeadband_t, imuRuntimeConfig_t

### Community 27 - "common/maths: ring_avg_u16_t"
Cohesion: 0.17
Nodes (9): arraySubInt32(), ring_avg_u16_get(), ring_avg_u16_init(), ring_avg_u16_push(), ring_avg_u16_reset(), resetBatteryConfig(), batteryInit(), ring_avg_u16_t (+1 more)

### Community 28 - "sensors + drivers: initialisation, accgyro_icm20948"
Cohesion: 0.07
Nodes (28): resetSensorAlignment(), sensorsSet(), icm20948AccDetect(), icm20948AccInit(), icm20948GyroDetect(), icm20948GyroInit(), icmSelectBank(), detectMpu() (+20 more)

### Community 29 - "io + API: oled_display, Oled"
Cohesion: 0.10
Nodes (27): Oled_Circle(), Oled_CircleOutline(), Oled_Erase(), Oled_Joysticks(), Oled_Number(), Oled_Pixel(), Oled_Rect(), Oled_RectOutline() (+19 more)

### Community 293 - "flight/mixer: feature()"
Cohesion: 0.11
Nodes (22): canUseBlackboxWithCurrentConfiguration(), initBlackbox(), feature(), pwmCompleteOneshotMotorUpdate(), pwmWriteMotor(), pwmWriteServo(), failsafeIsActive(), determineServoMiddleOrForwardFromChannel() (+14 more)

### Community 30 - "StdPeriph/rtc: ErrorStatus"
Cohesion: 0.06
Nodes (37): RTC_AlarmCmd(), RTC_AlarmStructInit(), RTC_Bcd2ToByte(), RTC_BypassShadowCmd(), RTC_ByteToBcd2(), RTC_CalibOutputCmd(), RTC_DateStructInit(), RTC_DeInit() (+29 more)

### Community 31 - "CMSIS/arm_math: arm_bilinear_interp_q15()"
Cohesion: 0.04
Nodes (54): arm_bilinear_interp_q15(), arm_pid_q31(), arm_bilinear_interp_instance_q15, arm_biquad_cas_df1_32x64_ins_q31, arm_biquad_cascade_df2T_instance_f32, arm_biquad_casd_df1_inst_f32, arm_biquad_casd_df1_inst_q15, arm_biquad_casd_df1_inst_q31 (+46 more)

### Community 32 - "rx: crsf, xbus"
Cohesion: 0.08
Nodes (30): userRCassert(), RcCommand_get(), RcCommand_Set(), RcData_Get(), micros(), warningLedRefresh(), warningLedUpdate(), crsfCrc8() (+22 more)

### Community 33 - "StdPeriph/i2c: I2C_TypeDef"
Cohesion: 0.07
Nodes (65): RCC_GetITStatus(), DMA_GetITStatus(), I2C_10BitAddressHeaderCmd(), I2C_10BitAddressingModeCmd(), I2C_AcknowledgeConfig(), I2C_AutoEndCmd(), I2C_CalculatePEC(), I2C_ClearFlag() (+57 more)

### Community 34 - "StdPeriph/usart: FunctionalState"
Cohesion: 0.05
Nodes (66): RCC_APB2PeriphResetCmd(), SYSCFG_DMAChannelRemapConfig(), SYSCFG_GetFlagStatus(), SYSCFG_I2CFastModePlusConfig(), SYSCFG_ITConfig(), SYSCFG_TriggerRemapConfig(), SYSCFG_USBInterruptLineRemapCmd(), USART_AddressDetectionConfig() (+58 more)

### Community 35 - "USB-FS: regs, sil"
Cohesion: 0.19
Nodes (13): DataStageIn(), DataStageOut(), PMAToUserBufferCopy(), UserToPMABufferCopy(), GetEPRxAddr(), GetEPRxCount(), GetEPTxAddr(), SetEPTxCount() (+5 more)

### Community 36 - "CMSIS/cm3: __INLINE"
Cohesion: 0.08
Nodes (41): __CLREX(), __get_BASEPRI(), __set_BASEPRI(), __get_PRIMASK(), __set_PRIMASK(), __get_FAULTMASK(), __set_FAULTMASK(), __get_CONTROL() (+33 more)

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
Cohesion: 0.17
Nodes (22): apmBaroCalculateAltitude(), apmBaroCalibrate(), apmBaroCalibrationRead(), apmBaroRead(), apmBaroUpdate(), baroCalculateAltitude(), baroCalibrate(), baroCompensationPa() (+14 more)

### Community 41 - "flight/navigation: onGpsNewData()"
Cohesion: 0.16
Nodes (27): get_PIDProfile(), set_PIDProfile(), check_missed_wp(), get_D(), get_I(), get_P(), GPS_calc_desired_speed(), GPS_calc_location_error() (+19 more)

### Community 42 - "drivers: serial_1wire, system"
Cohesion: 0.16
Nodes (15): i2cInit(), gpioInit(), initInverter(), ledInit(), pwmGPIOConfig(), pwmGPIOConfig(), cycleCounterInit(), enableGPIOPowerUsageAndNoiseReductions() (+7 more)

### Community 43 - "CMSIS/cm4_simd: __STATIC_INLINE"
Cohesion: 0.08
Nodes (49): __QADD(), __QADD16(), __QADD8(), __QASX(), __QSAX(), __QSUB(), __QSUB16(), __QSUB8() (+41 more)

### Community 45 - "StdPeriph: pwr, wwdg"
Cohesion: 0.08
Nodes (10): I2C_DeInit(), PWR_BackupAccessCmd(), PWR_DeInit(), PWR_GetFlagStatus(), PWR_PVDCmd(), PWR_WakeUpPinCmd(), RCC_APB1PeriphResetCmd(), WWDG_DeInit() (+2 more)

### Community 46 - "config/config: resetConf()"
Cohesion: 0.12
Nodes (12): featureClearAll(), resetConf(), resetControlRateConfig(), resetEscAndServoConfig(), resetFlight3DConfig(), resetMixerConfig(), resetPidProfile(), resetRcControlsConfig() (+4 more)

### Community 47 - "config + flight: config, runtime_config"
Cohesion: 0.16
Nodes (14): changeControlRateProfile(), changeProfile(), readEEPROM(), setControlRateProfile(), setProfile(), disableFlightMode(), enableFlightMode(), failsafeOnRxResume() (+6 more)

### Community 48 - "io/serial_cli: cliPrint()"
Cohesion: 0.11
Nodes (51): fastA2F(), ftoa(), itoa(), featureMask(), getCurrentControlRateProfile(), sensorsMask(), cliAdjustmentRange(), cliAux() (+43 more)

### Community 49 - "flight/altitudehold: apmCalculateEstimatedAltitude()"
Cohesion: 0.19
Nodes (13): applyDeadband(), addHistPositionBaseEstZ(), altLaserFusion(), AltRst(), apmCalculateEstimatedAltitude(), calculateEstimatedAltitude(), checkReading(), kalmanFilterInit() (+5 more)

### Community 5 - "VL53L1X/register_funcs: VL53L1_ReadMulti()"
Cohesion: 0.06
Nodes (86): VL53L1_disable_firmware(), VL53L1_enable_firmware(), VL53L1_i2c_decode_int32_t(), VL53L1_i2c_decode_uint16_t(), VL53L1_i2c_decode_uint32_t(), VL53L1_i2c_encode_int32_t(), VL53L1_i2c_encode_uint16_t(), VL53L1_i2c_encode_uint32_t() (+78 more)

### Community 50 - "config + API: config, RxConfig"
Cohesion: 0.18
Nodes (12): plutoRxConfig(), Receiver_Config_Mode_Dev(), Receiver_Mode(), featureConfigured(), featureSet(), handleOneshotFeatureChangeOnRestart(), resetSerialConfig(), validateAndFixConfig() (+4 more)

### Community 52 - "flight/posEstimate: imuResetAccelerationSum()"
Cohesion: 0.20
Nodes (12): altitudeHoldGetEstimatedAltitude(), dcmBodyToEarth3D(), imuResetAccelerationSum(), calculateSensorFlow(), runFlowHold(), updateHeightEstimate(), addHistPositionBaseEstXY(), checkPosition() (+4 more)

### Community 53 - "io/oled_display: Oled_FillRoundedRect()"
Cohesion: 0.24
Nodes (11): Oled_RobotEyes(), Oled_RoundEyes(), Oled_RoundRect(), clampPupil(), Oled_DrawBoxyEye(), Oled_DrawBoxyEyeFilled(), Oled_DrawBoxyEyeOutline(), Oled_DrawEye() (+3 more)

### Community 54 - "flight + io: pid, posControl"
Cohesion: 0.14
Nodes (28): Peripheral_Write(), Servo_Write(), Interval::set(), constrain(), map_i32(), activateControlRateConfig(), filterApplyPt1(), calculate_Gtune() (+20 more)

### Community 56 - "telemetry: serial, frsky"
Cohesion: 0.24
Nodes (6): determinePortSharing(), initFrSkyTelemetry(), initHoTTTelemetry(), initMSPTelemetry(), initSmartPortTelemetry(), telemetryInit()

### Community 57 - "StdPeriph/hrtim: HRTIM_TypeDef"
Cohesion: 0.08
Nodes (46): HRTIM_BurstDMAConfig(), HRTIM_BurstModeCtl(), HRTIM_ClearCommonFlag(), HRTIM_ClearCommonITPendingBit(), HRTIM_ClearFlag(), HRTIM_ClearITPendingBit(), HRTIM_DeInit(), HRTIM_DLLCalibrationStart() (+38 more)

### Community 58 - "flight/imu: drift_correction_yaw()"
Cohesion: 0.27
Nodes (10): radians(), calculate_heading(), dcmFromEuler(), dcmMulTranspose(), dcmMulXY(), drift_correction(), drift_correction_yaw(), pGain() (+2 more)

### Community 59 - "flight/imu: imuUpdate()"
Cohesion: 0.24
Nodes (10): safe_asin(), bodyDcmToEuler(), check_matrix(), euler_angles(), imuUpdate(), matrix_update(), normalize(), renorm() (+2 more)

### Community 6 - "API/X1Ranging: laser_e"
Cohesion: 0.13
Nodes (20): altRequest(), isLedUsable(), isValidGpio(), ledInit(), ledWrite(), X1Ranging_P::disableOA(), X1Ranging_P::getRange(), X1Ranging_P::getStatus() (+12 more)

### Community 60 - "API/RxConfig: Receiver_Aux_Config()"
Cohesion: 0.43
Nodes (8): Receiver_Aux_Config(), Receiver_Config_Arm(), Receiver_Config_Mode_Angle(), Receiver_Config_Mode_Baro(), Receiver_Config_Mode_HeadFree(), Receiver_Config_Mode_Mag(), flight_mode, rx_channel_e

### Community 62 - "common/atomic: __basepriSetMemRetVal()"
Cohesion: 0.38
Nodes (4): __basepriSetMemRetVal(), __basepriSetRetVal(), __set_BASEPRI_MAX(), __set_BASEPRI_MAX_nb()

### Community 64 - "API/FC-Config-Setpoints: angle_e"
Cohesion: 0.33
Nodes (6): DesiredAngle_setRelative(), getDesiredAngle(), getDesiredRate(), setDesiredAngle(), setDesiredRate(), angle_e

### Community 65 - "sensors + config: barometer, config"
Cohesion: 0.40
Nodes (4): resetBarometerConfig(), configureAltitudeHold(), useBarometerConfig(), barometerConfig_t

### Community 66 - "flight/navigation: navigationInit()"
Cohesion: 0.50
Nodes (5): resetGpsProfile(), gpsUsePIDs(), gpsUseProfile(), navigationInit(), gpsProfile_t

### Community 67 - "drivers + common: light_ws2811strip, colorconversion"
Cohesion: 0.18
Nodes (10): RGB_Init(), hsvToRgb24(), fastUpdateLEDDMABuffer(), ws2811LedStripSetPin(), updateLEDDMABuffer(), ws2811LedStripInit(), ws2811UpdateStrip(), ledStripEnable() (+2 more)

### Community 7 - "VL53L0X/api: VL53L0X_RdByte()"
Cohesion: 0.17
Nodes (19): VL53L0X_device_read_strobe(), VL53L0X_get_info_from_device(), VL53L0X_measurement_poll_for_completion(), VL53L0X_check_part_used(), VL53L0X_get_device_info(), VL53L0X_GetDeviceInfo(), VL53L0X_GetFractionEnable(), VL53L0X_GetInterruptMaskStatus() (+11 more)

### Community 71 - "CMSIS/rcc: RCC_GetFlagStatus()"
Cohesion: 0.05
Nodes (15): RCC_AHBPeriphClockCmd(), RCC_APB1PeriphClockCmd(), RCC_APB1PeriphResetCmd(), RCC_APB2PeriphClockCmd(), RCC_APB2PeriphResetCmd(), RCC_BackupResetCmd(), RCC_ClockSecuritySystemCmd(), RCC_GetClocksFreq() (+7 more)

### Community 72 - "VL53L1X: wait, platform"
Cohesion: 0.10
Nodes (22): VL53L1_software_reset(), VL53L1_init_ll_driver_state(), VL53L1_is_firmware_ready_silicon(), VL53L1_is_boot_complete(), VL53L1_is_firmware_ready(), VL53L1_is_new_data_ready(), VL53L1_poll_for_boot_completion(), VL53L1_poll_for_firmware_ready() (+14 more)

### Community 77 - "StdPeriph/can: CAN_TypeDef"
Cohesion: 0.11
Nodes (29): CAN_CancelTransmit(), CAN_ClearFlag(), CAN_ClearITPendingBit(), CAN_DBGFreeze(), CAN_DeInit(), CAN_FIFORelease(), CAN_FilterInit(), CAN_GetFlagStatus() (+21 more)

### Community 78 - "VL53L1X/def: ConvertModeFromLLD()"
Cohesion: 0.06
Nodes (30): ConvertModeFromLLD(), ConvertModeToLLD(), VL53L1_get_limit_check_info(), VL53L1_get_pal_state_string(), VL53L1_get_range_status_string(), VL53L1_get_sequence_steps_info(), VL53L1_GetLimitCheckInfo(), VL53L1_GetPalErrorString() (+22 more)

### Community 8 - "StdPeriph/tim: TIM_TypeDef"
Cohesion: 0.05
Nodes (82): TIM_ARRPreloadConfig(), TIM_Break1Cmd(), TIM_Break1Config(), TIM_Break2Cmd(), TIM_Break2Config(), TIM_CCPreloadControl(), TIM_CCxCmd(), TIM_CCxNCmd() (+74 more)

### Community 82 - "StdPeriph/spi: SPI_TypeDef"
Cohesion: 0.13
Nodes (28): I2S_Cmd(), I2S_FullDuplexConfig(), I2S_Init(), I2S_StructInit(), SPI_BiDirectionalLineConfig(), SPI_CalculateCRC(), SPI_CRCLengthConfig(), SPI_DataSizeConfig() (+20 more)

### Community 83 - "VL53L0X/api_core: get_sequence_step_timeout()"
Cohesion: 0.16
Nodes (28): get_sequence_step_timeout(), set_sequence_step_timeout(), VL53L0X_calc_dmax(), VL53L0X_calc_macro_period_ps(), VL53L0X_calc_sigma_estimate(), VL53L0X_calc_timeout_mclks(), VL53L0X_calc_timeout_us(), VL53L0X_decode_timeout() (+20 more)

### Community 9 - "blackbox/blackbox: blackboxWrite()"
Cohesion: 0.07
Nodes (52): blackboxAdvanceIterationTimers(), blackboxBuildConditionCache(), blackboxCheckAndLogArmingBeep(), blackboxIsOnlyLoggingIntraframes(), blackboxLogEvent(), blackboxLogIteration(), blackboxSetState(), blackboxShouldLogIFrame() (+44 more)

### Community 91 - "CMSIS/cm4: __STATIC_INLINE"
Cohesion: 0.09
Nodes (30): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), NVIC_DisableIRQ(), NVIC_EnableIRQ(), NVIC_EncodePriority() (+22 more)

### Community 94 - "CMSIS/cm3: __STATIC_INLINE"
Cohesion: 0.10
Nodes (29): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), NVIC_DisableIRQ(), NVIC_EnableIRQ(), NVIC_EncodePriority() (+21 more)

### Community 95 - "CMSIS/sc300: __STATIC_INLINE"
Cohesion: 0.10
Nodes (29): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), NVIC_DisableIRQ(), NVIC_EnableIRQ(), NVIC_EncodePriority() (+21 more)

### Community 96 - "vcp: prop, int"
Cohesion: 0.09
Nodes (5): Standard_GetDescriptorData(), Virtual_Com_Port_GetConfigDescriptor(), Virtual_Com_Port_GetDeviceDescriptor(), Virtual_Com_Port_GetStringDescriptor(), ONE_DESCRIPTOR

### Community 97 - "VL53L0X: i2c_platform, platform"
Cohesion: 0.08
Nodes (20): VL53L0X_load_tuning_settings(), VL53L0X_read_byte(), VL53L0X_read_multi(), VL53L0X_read_word(), VL53L0X_write_byte(), VL53L0X_write_dword(), VL53L0X_write_multi(), VL53L0X_write_word() (+12 more)

### Community 99 - "io/display: updateDisplay()"
Cohesion: 0.14
Nodes (32): putcp(), tfp_sprintf(), getControlRateConfig(), getCurrentProfile(), i2c_OLED_send_char(), i2c_OLED_send_string(), i2c_OLED_set_line(), i2c_OLED_set_xy() (+24 more)

### Community 243 - "StdPeriph/gpio: BitAction"
Cohesion: 0.29
Nodes (6): BitAction, GPIO_InitTypeDef, GPIOMode_TypeDef, GPIOOType_TypeDef, GPIOPuPd_TypeDef, GPIOSpeed_TypeDef

### Community 61 - "CMSIS/gpio: BitAction"
Cohesion: 0.29
Nodes (6): BitAction, GPIO_InitTypeDef, GPIOMode_TypeDef, GPIOOType_TypeDef, GPIOPuPd_TypeDef, GPIOSpeed_TypeDef

## Knowledge Gaps
- **631 isolated node(s):** `IN_DATA`, `LAST_IN_DATA`, `LAST_OUT_DATA`, `OUT_DATA`, `PAUSE` (+626 more)
  These have ≤1 connection - possible missing edges. (Counts symbols only; 1300 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **18 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `FunctionalState` connect `StdPeriph/usart: FunctionalState` to `StdPeriph/tim: TIM_TypeDef`, `StdPeriph/dac: DAC_TypeDef`, `CMSIS/stm32f30x: DBGMCU_APB1PeriphConfig()`, `CMSIS + StdPeriph: gpio, rcc`, `StdPeriph/adc: ADC_TypeDef`, `drivers: pwm_output, tim`, `drivers/timer: timerHardware_t`, `drivers + StdPeriph: bus_spi, spi`, `vcp/pwr: USB_Istr()`, `StdPeriph/rcc: RCC_AHBPeriphResetCmd()`, `StdPeriph/tim: ws2811LedStripHardwareInit()`, `StdPeriph/rtc: ErrorStatus`, `StdPeriph/opamp: OPAMP_InitTypeDef`, `StdPeriph/i2c: I2C_TypeDef`, `drivers + StdPeriph: dma, serial`, `StdPeriph/crc: IRQn`, `StdPeriph: pwr, wwdg`, `StdPeriph/hrtim: HRTIM_TypeDef`, `CMSIS/rcc: RCC_GetFlagStatus()`, `StdPeriph/can: CAN_TypeDef`, `StdPeriph/spi: SPI_TypeDef`, `StdPeriph/comp: COMP_InitTypeDef`, `StdPeriph/flash: FLASH_WaitForLastOperation()`?**
  _High betweenness centrality (0.066) - this node is a cross-community bridge._
- **What connects `IN_DATA`, `LAST_IN_DATA`, `LAST_OUT_DATA` to the rest of the system?**
  _631 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `USB-FS/core: DEVICE_INFO` be split into smaller, more focused modules?**
  _Cohesion score 0.052564102564102565 - nodes in this community are weakly interconnected._
- **Why does `VL53L1_Error` connect `VL53L1X: api, api_core` to `VL53L1X/register_funcs: VL53L1_ReadMulti()`, `VL53L1X: ll_def, api_debug`, `VL53L1X: wait, platform`, `drivers/tof_manager_l1x: tofId_e`, `VL53L1X/def: ConvertModeFromLLD()`, `VL53L1X/core: VL53L1_WrByte()`, `VL53L1X/api_preset_modes: VL53L1_set_preset_mode()`, `VL53L1X/register_funcs: VL53L1_init_and_start_range()`, `VL53L1X: api_core, core`?**
  _High betweenness centrality (0.048) - this node is a cross-community bridge._
- **Should `drivers/sc18is602b: SC18IS602B` be split into smaller, more focused modules?**
  _Cohesion score 0.06747638326585695 - nodes in this community are weakly interconnected._
- **Why does `SPI_TypeDef` connect `StdPeriph/spi: SPI_TypeDef` to `drivers + StdPeriph: bus_spi, spi`, `CMSIS/stm32f30x: DBGMCU_APB1PeriphConfig()`?**
  _High betweenness centrality (0.030) - this node is a cross-community bridge._
- **Should `io/serial: serialPortFunction_e` be split into smaller, more focused modules?**
  _Cohesion score 0.08870967741935484 - nodes in this community are weakly interconnected._