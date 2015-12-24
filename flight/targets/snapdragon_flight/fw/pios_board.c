/**
 ******************************************************************************
 * @addtogroup TauLabsTargets Tau Labs Targets
 * @{
 * @addtogroup SnapdragonFlight Snapdragon Flight bootloader
 * @{
 *
 * @file       pios_board.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @author     Kenn Sebesta, Copyright (C) 2015
 * @brief      Board initialization file
 * @see        The GNU Public License (GPL) Version 3
 * 
 *****************************************************************************/
/* 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation; either version 3 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for more details.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with this program; if not, write to the Free Software Foundation, Inc., 
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/* Pull in the board-specific static HW definitions.
 * Including .c files is a bit ugly but this allows all of
 * the HW definitions to be const and static to limit their
 * scope.  
 *
 * NOTE: THIS IS THE ONLY PLACE THAT SHOULD EVER INCLUDE THIS FILE
 */
#include "board_hw_defs.c"

#include <pios.h>
#include <pios_hal.h>
#include <openpilot.h>
#include <uavobjectsinit.h>
#include "hwsnapdragonflight.h"
#include "manualcontrolsettings.h"
#include "modulesettings.h"

/**
 * Sensor configurations
 */
#if defined(PIOS_INCLUDE_MPU9250_SPI)
#include "pios_mpu9250.h"
static const struct pios_exti_cfg pios_exti_mpu9250_cfg __exti_config = {
	.vector = PIOS_MPU9250_IRQHandler,
	.line = EXTI_Line8,
	.pin = {
		.gpio = GPIOD,
		.init = {
			.GPIO_Pin = GPIO_Pin_8,
			.GPIO_Speed = GPIO_Speed_100MHz,
			.GPIO_Mode = GPIO_Mode_IN,
			.GPIO_OType = GPIO_OType_OD,
			.GPIO_PuPd = GPIO_PuPd_NOPULL,
		},
	},
	.irq = {
		.init = {
			.NVIC_IRQChannel = EXTI9_5_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_HIGH,
			.NVIC_IRQChannelSubPriority = 0,
			.NVIC_IRQChannelCmd = ENABLE,
		},
	},
	.exti = {
		.init = {
			.EXTI_Line = EXTI_Line8, // matches above GPIO pin
			.EXTI_Mode = EXTI_Mode_Interrupt,
			.EXTI_Trigger = EXTI_Trigger_Rising,
			.EXTI_LineCmd = ENABLE,
		},
	},
};

static struct pios_mpu9250_cfg pios_mpu9250_cfg = {
	.exti_cfg = &pios_exti_mpu9250_cfg,
	.default_samplerate = 500,
	.interrupt_cfg = PIOS_MPU60X0_INT_CLR_ANYRD,
	.use_magnetometer = true,
	.default_gyro_filter = PIOS_MPU9250_GYRO_LOWPASS_184_HZ,
	.default_accel_filter = PIOS_MPU9250_ACCEL_LOWPASS_184_HZ,
	.orientation = PIOS_MPU9250_TOP_0DEG
};
#endif /* PIOS_INCLUDE_MPU9250_SPI */

/**
 * Configuration for the MS5611 chip
 */
#if defined(PIOS_INCLUDE_MS5611_SPI)
#include "pios_ms5611_priv.h"
static const struct pios_ms5611_cfg pios_ms5611_cfg = {
	.oversampling = MS5611_OSR_1024,
	.temperature_interleaving = 1,
};
#endif /* PIOS_INCLUDE_MS5611_SPI */

uintptr_t pios_com_spiflash_logging_id;
uintptr_t pios_internal_adc_id = 0;
uintptr_t pios_uavo_settings_fs_id;
uintptr_t pios_waypoints_settings_fs_id;

uintptr_t streamfs_id;
uintptr_t external_i2c_adapter_id = 0;

/**
 * Indicate a target-specific error code when a component fails to initialize
 * 1 pulse - flash chip
 * 2 pulses - MPU9250
 * 4 pulses - MS5611
 * 6 pulses - external mag
 */
static void panic(int32_t code) {
	PIOS_HAL_Panic(PIOS_LED_ALARM, code);
}

/**
 * PIOS_Board_Init()
 * initializes all the core subsystems on this specific hardware
 * called from System/openpilot.c
 */

#include <pios_board_info.h>

/**
 * Check the brown out reset threshold is 2.7 volts and if not
 * resets it.  This solves an issue that can prevent boards
 * powering up with some BEC
 */
void check_bor()
{
    uint8_t bor = FLASH_OB_GetBOR();

    if (bor != OB_BOR_LEVEL3) {
        FLASH_OB_Unlock();
        FLASH_OB_BORConfig(OB_BOR_LEVEL3);
        FLASH_OB_Launch();
        while (FLASH_WaitForLastOperation() == FLASH_BUSY) {
            ;
        }
        FLASH_OB_Lock();
        while (FLASH_WaitForLastOperation() == FLASH_BUSY) {
            ;
        }
    }
}

void PIOS_Board_Init(void)
{
	// Check brownout threshold
	check_bor();

	/* Delay system */
	PIOS_DELAY_Init();

	const struct pios_board_info * bdinfo = &pios_board_info_blob;

#if defined(PIOS_INCLUDE_LED)
	PIOS_LED_Init(&pios_led_cfg);
#endif	/* PIOS_INCLUDE_LED */

#if defined(PIOS_INCLUDE_SPI)
	if (PIOS_SPI_Init(&pios_spi1_id, &pios_spi1_cfg)) {
		PIOS_DEBUG_Assert(0);
	}

	if (PIOS_SPI_Init(&pios_spi2_id, &pios_spi2_cfg)) {
		PIOS_DEBUG_Assert(0);
	}

	if (PIOS_SPI_Init(&pios_spi3_id, &pios_spi3_cfg)) {
		PIOS_DEBUG_Assert(0);
	}
#endif


#if defined(PIOS_INCLUDE_FLASH)
	/* Inititialize all flash drivers */
#if defined(PIOS_INCLUDE_FLASH_JEDEC)
	if (PIOS_Flash_Jedec_Init(&pios_external_flash_id, pios_spi3_id, 0, &flash_mx25_cfg) != 0)
		panic(1);
#endif /* PIOS_INCLUDE_FLASH_JEDEC */
	if (PIOS_Flash_Internal_Init(&pios_internal_flash_id, &flash_internal_cfg) != 0)
		panic(1);

	/* Register the partition table */
	const struct pios_flash_partition * flash_partition_table;
	uint32_t num_partitions;
	flash_partition_table = PIOS_BOARD_HW_DEFS_GetPartitionTable(bdinfo->board_rev, &num_partitions);
	PIOS_FLASH_register_partition_table(flash_partition_table, num_partitions);

	/* Mount all filesystems */
	if (PIOS_FLASHFS_Logfs_Init(&pios_uavo_settings_fs_id, &flashfs_settings_cfg, FLASH_PARTITION_LABEL_SETTINGS) != 0)
		panic(1);
#if defined(PIOS_INCLUDE_FLASH_JEDEC)
	if (PIOS_FLASHFS_Logfs_Init(&pios_waypoints_settings_fs_id, &flashfs_waypoints_cfg, FLASH_PARTITION_LABEL_WAYPOINTS) != 0)
		panic(1);
#endif /* PIOS_INCLUDE_FLASH_JEDEC */
#if defined(ERASE_FLASH)
	PIOS_FLASHFS_Format(pios_uavo_settings_fs_id);
#endif

#endif	/* PIOS_INCLUDE_FLASH */

	/* Initialize UAVObject libraries */
	EventDispatcherInitialize();
	UAVObjInitialize();

	HwSnapdragonFlightInitialize();
	ModuleSettingsInitialize();

#if defined(PIOS_INCLUDE_RTC)
	/* Initialize the real-time clock and its associated tick */
	PIOS_RTC_Init(&pios_rtc_main_cfg);
#endif

#ifndef ERASE_FLASH
	/* Initialize watchdog as early as possible to catch faults during init
	 * but do it only if there is no debugger connected
	 */
	if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) == 0) {
		PIOS_WDG_Init();
	}
#endif

	/* Initialize the alarms library */
	AlarmsInitialize();

	/* Initialize the task monitor library */
	TaskMonitorInitialize();

	/* IAP System Setup */
	PIOS_IAP_Init();
	uint16_t boot_count = PIOS_IAP_ReadBootCount();
	if (boot_count < 3) {
		PIOS_IAP_WriteBootCount(++boot_count);
		AlarmsClear(SYSTEMALARMS_ALARM_BOOTFAULT);
	} else {
		/* Too many failed boot attempts, force hw config to defaults */
		HwSnapdragonFlightSetDefaults(HwSnapdragonFlightHandle(), 0);
		ModuleSettingsSetDefaults(ModuleSettingsHandle(),0);
		AlarmsSet(SYSTEMALARMS_ALARM_BOOTFAULT, SYSTEMALARMS_ALARM_CRITICAL);
	}

#if defined(PIOS_INCLUDE_USB)
	/* Initialize board specific USB data */
	PIOS_USB_BOARD_DATA_Init();

	/* Flags to determine if various USB interfaces are advertised */
	bool usb_hid_present = false;
	bool usb_cdc_present = false;

#if defined(PIOS_INCLUDE_USB_CDC)
	if (PIOS_USB_DESC_HID_CDC_Init()) {
		PIOS_Assert(0);
	}
	usb_hid_present = true;
	usb_cdc_present = true;
#else
	if (PIOS_USB_DESC_HID_ONLY_Init()) {
		PIOS_Assert(0);
	}
	usb_hid_present = true;
#endif

	uintptr_t pios_usb_id;
	PIOS_USB_Init(&pios_usb_id, &pios_usb_main_cfg);

#if defined(PIOS_INCLUDE_USB_CDC)

	uint8_t hw_usb_vcpport;
	/* Configure the USB VCP port */
	HwSnapdragonFlightUSB_VCPPortGet(&hw_usb_vcpport);

	if (!usb_cdc_present) {
		/* Force VCP port function to disabled if we haven't advertised VCP in our USB descriptor */
		hw_usb_vcpport = HWSNAPDRAGONFLIGHT_USB_VCPPORT_DISABLED;
	}

	PIOS_HAL_ConfigureCDC(hw_usb_vcpport, pios_usb_id, &pios_usb_cdc_cfg);

#endif	/* PIOS_INCLUDE_USB_CDC */

#if defined(PIOS_INCLUDE_USB_HID)
	/* Configure the usb HID port */
	uint8_t hw_usb_hidport;
	HwSnapdragonFlightUSB_HIDPortGet(&hw_usb_hidport);

	if (!usb_hid_present) {
		/* Force HID port function to disabled if we haven't advertised HID in our USB descriptor */
		hw_usb_hidport = HWSNAPDRAGONFLIGHT_USB_HIDPORT_DISABLED;
	}

	PIOS_HAL_ConfigureHID(hw_usb_hidport, pios_usb_id, &pios_usb_hid_cfg);

#endif	/* PIOS_INCLUDE_USB_HID */

	if (usb_hid_present || usb_cdc_present) {
		PIOS_USBHOOK_Activate();
	}
#endif	/* PIOS_INCLUDE_USB */

	/* Configure the IO ports */
	uint8_t hw_DSMxMode;
	HwSnapdragonFlightDSMxModeGet(&hw_DSMxMode);

	/* init sensor queue registration */
	PIOS_SENSORS_Init();


	/* UART2 Port */
	uint8_t hw_mainport;
	HwSnapdragonFlightMainPortGet(&hw_mainport);

	PIOS_HAL_ConfigurePort(hw_mainport,           // port type protocol
			&pios_mainport_cfg,                    // usart_port_cfg
			&pios_mainport_cfg,                    // frsky usart_port_cfg
			&pios_usart_com_driver,              // com_driver
			NULL,                                // i2c_id
			NULL,                                // i2c_cfg
			NULL,                                // i2c_cfg
			NULL,                                // pwm_cfg
			PIOS_LED_ALARM,                      // led_id
			NULL,                                // usart_dsm_hsum_cfg
			NULL,                                // dsm_cfg
			hw_DSMxMode,                         // dsm_mode
			NULL,                                // sbus_rcvr_cfg
			NULL,                                // sbus_cfg
			false);                              // sbus_toggle

	/* Configure the PPM receiver port */
#if defined(PIOS_INCLUDE_PPM)
	PIOS_TIM_InitClock(&tim_1_cfg);

	uintptr_t pios_ppm_id;
	PIOS_PPM_Init(&pios_ppm_id, &pios_ppm_cfg);

	uintptr_t pios_ppm_rcvr_id;
	if (PIOS_RCVR_Init(&pios_ppm_rcvr_id, &pios_ppm_rcvr_driver, pios_ppm_id)) {
		PIOS_Assert(0);
	}
	pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_PPM] = pios_ppm_rcvr_id;
#endif /* PIOS_INCLUDE_PPM */

#if defined(PIOS_INCLUDE_GCSRCVR)
	GCSReceiverInitialize();
	uintptr_t pios_gcsrcvr_id;
	PIOS_GCSRCVR_Init(&pios_gcsrcvr_id);
	uintptr_t pios_gcsrcvr_rcvr_id;
	if (PIOS_RCVR_Init(&pios_gcsrcvr_rcvr_id, &pios_gcsrcvr_rcvr_driver, pios_gcsrcvr_id)) {
		PIOS_Assert(0);
	}
	pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_GCS] = pios_gcsrcvr_rcvr_id;
#endif	/* PIOS_INCLUDE_GCSRCVR */

	/* Configure the I2C1 bus */
	if (PIOS_I2C_Init(&pios_i2c1_adapter_id, &pios_i2c1_adapter_cfg)) {
		PIOS_DEBUG_Assert(0);
	}


	/* Configure the PWM outputs */
#ifdef PIOS_INCLUDE_SERVO
	PIOS_Servo_Init(&pios_pwm_out_cfg);
#endif


	PIOS_WDG_Clear();
	PIOS_DELAY_WaitmS(200);
	PIOS_WDG_Clear();

#if defined(PIOS_INCLUDE_GPIO)
	PIOS_GPIO_Init();
#endif

	/* Configure Barometer */
#if defined(PIOS_INCLUDE_MS5611_SPI)
		if (PIOS_MS5611_SPI_Init(pios_spi2_id, 0, &pios_ms5611_cfg) != 0){
			panic(5);
		}
#endif /* PIOS_INCLUDE_MS5611_SPI */

	/* Configure Magnetometer */
	uint8_t hw_mag;
	HwSnapdragonFlightMagnetometerGet(&hw_mag);
	switch (hw_mag) {
		case HWSNAPDRAGONFLIGHT_MAGNETOMETER_EXTERNALAUXI2C:
#if defined(PIOS_INCLUDE_HMC5883)
			if (PIOS_HMC5883_Init(pios_i2c1_adapter_id, &pios_hmc5883_cfg) != 0){
				panic(6);
			}
			if (PIOS_HMC5883_Test() != 0){
				panic(6);
			}
#endif /* PIOS_INCLUDE_HMC5883 */
			break;
		case HWSNAPDRAGONFLIGHT_MAGNETOMETER_INTERNAL:
#if defined(PIOS_INCLUDE_MPU9250_SPI)
			pios_mpu9250_cfg.use_magnetometer = true;
#endif /* PIOS_INCLUDE_MPU9250_SPI */
	}

	/* Configure IMU */
	uint8_t hw_imu = 0;
//	HwSnapdragonFlightIMUGet(&hw_imu);
	switch (hw_imu) {
		case /*HWSNAPDRAGONFLIGHT_IMU_MPU9250*/0:
		{
#if defined(PIOS_INCLUDE_MPU9250_SPI)
			if (PIOS_MPU9250_SPI_Init(pios_spi1_id, 0, &pios_mpu9250_cfg) != 0)
				panic(2);

			// To be safe map from UAVO enum to driver enum
			uint8_t hw_gyro_range;
			HwSnapdragonFlightGyroRangeGet(&hw_gyro_range);
			switch(hw_gyro_range) {
				case HWSNAPDRAGONFLIGHT_GYRORANGE_250:
					PIOS_MPU9250_SetGyroRange(PIOS_MPU60X0_SCALE_250_DEG);
					break;
				case HWSNAPDRAGONFLIGHT_GYRORANGE_500:
					PIOS_MPU9250_SetGyroRange(PIOS_MPU60X0_SCALE_500_DEG);
					break;
				case HWSNAPDRAGONFLIGHT_GYRORANGE_1000:
					PIOS_MPU9250_SetGyroRange(PIOS_MPU60X0_SCALE_1000_DEG);
					break;
				case HWSNAPDRAGONFLIGHT_GYRORANGE_2000:
					PIOS_MPU9250_SetGyroRange(PIOS_MPU60X0_SCALE_2000_DEG);
					break;
			}

			uint8_t hw_accel_range;
			HwSnapdragonFlightAccelRangeGet(&hw_accel_range);
			switch(hw_accel_range) {
				case HWSNAPDRAGONFLIGHT_ACCELRANGE_2G:
					PIOS_MPU9250_SetAccelRange(PIOS_MPU60X0_ACCEL_2G);
					break;
				case HWSNAPDRAGONFLIGHT_ACCELRANGE_4G:
					PIOS_MPU9250_SetAccelRange(PIOS_MPU60X0_ACCEL_4G);
					break;
				case HWSNAPDRAGONFLIGHT_ACCELRANGE_8G:
					PIOS_MPU9250_SetAccelRange(PIOS_MPU60X0_ACCEL_8G);
					break;
				case HWSNAPDRAGONFLIGHT_ACCELRANGE_16G:
					PIOS_MPU9250_SetAccelRange(PIOS_MPU60X0_ACCEL_16G);
					break;
			}

			// the filter has to be set before rate else divisor calculation will fail
			uint8_t hw_mpu9250_dlpf;
			HwSnapdragonFlightMPU9250GyroLPFGet(&hw_mpu9250_dlpf);
			enum pios_mpu9250_gyro_filter mpu9250_gyro_lpf = \
				(hw_mpu9250_dlpf == HWSNAPDRAGONFLIGHT_MPU9250GYROLPF_250) ? PIOS_MPU9250_GYRO_LOWPASS_250_HZ : \
				(hw_mpu9250_dlpf == HWSNAPDRAGONFLIGHT_MPU9250GYROLPF_184) ? PIOS_MPU9250_GYRO_LOWPASS_184_HZ : \
				(hw_mpu9250_dlpf == HWSNAPDRAGONFLIGHT_MPU9250GYROLPF_92) ? PIOS_MPU9250_GYRO_LOWPASS_92_HZ : \
				(hw_mpu9250_dlpf == HWSNAPDRAGONFLIGHT_MPU9250GYROLPF_41) ? PIOS_MPU9250_GYRO_LOWPASS_41_HZ : \
				(hw_mpu9250_dlpf == HWSNAPDRAGONFLIGHT_MPU9250GYROLPF_20) ? PIOS_MPU9250_GYRO_LOWPASS_20_HZ : \
				(hw_mpu9250_dlpf == HWSNAPDRAGONFLIGHT_MPU9250GYROLPF_10) ? PIOS_MPU9250_GYRO_LOWPASS_10_HZ : \
				(hw_mpu9250_dlpf == HWSNAPDRAGONFLIGHT_MPU9250GYROLPF_5) ? PIOS_MPU9250_GYRO_LOWPASS_5_HZ : \
				pios_mpu9250_cfg.default_gyro_filter;
			PIOS_MPU9250_SetGyroLPF(mpu9250_gyro_lpf);

			HwSnapdragonFlightMPU9250AccelLPFGet(&hw_mpu9250_dlpf);
			enum pios_mpu9250_accel_filter mpu9250_accel_lpf = \
				(hw_mpu9250_dlpf == HWSNAPDRAGONFLIGHT_MPU9250ACCELLPF_460) ? PIOS_MPU9250_ACCEL_LOWPASS_460_HZ : \
				(hw_mpu9250_dlpf == HWSNAPDRAGONFLIGHT_MPU9250ACCELLPF_184) ? PIOS_MPU9250_ACCEL_LOWPASS_184_HZ : \
				(hw_mpu9250_dlpf == HWSNAPDRAGONFLIGHT_MPU9250ACCELLPF_92) ? PIOS_MPU9250_ACCEL_LOWPASS_92_HZ : \
				(hw_mpu9250_dlpf == HWSNAPDRAGONFLIGHT_MPU9250ACCELLPF_41) ? PIOS_MPU9250_ACCEL_LOWPASS_41_HZ : \
				(hw_mpu9250_dlpf == HWSNAPDRAGONFLIGHT_MPU9250ACCELLPF_20) ? PIOS_MPU9250_ACCEL_LOWPASS_20_HZ : \
				(hw_mpu9250_dlpf == HWSNAPDRAGONFLIGHT_MPU9250ACCELLPF_10) ? PIOS_MPU9250_ACCEL_LOWPASS_10_HZ : \
				(hw_mpu9250_dlpf == HWSNAPDRAGONFLIGHT_MPU9250ACCELLPF_5) ? PIOS_MPU9250_ACCEL_LOWPASS_5_HZ : \
				pios_mpu9250_cfg.default_accel_filter;
			PIOS_MPU9250_SetAccelLPF(mpu9250_accel_lpf);

			uint8_t hw_mpu9250_samplerate;
			HwSnapdragonFlightMPU9250RateGet(&hw_mpu9250_samplerate);
			uint16_t mpu9250_samplerate = \
				(hw_mpu9250_samplerate == HWSNAPDRAGONFLIGHT_MPU9250RATE_200) ? 200 : \
				(hw_mpu9250_samplerate == HWSNAPDRAGONFLIGHT_MPU9250RATE_250) ? 250 : \
				(hw_mpu9250_samplerate == HWSNAPDRAGONFLIGHT_MPU9250RATE_333) ? 333 : \
				(hw_mpu9250_samplerate == HWSNAPDRAGONFLIGHT_MPU9250RATE_500) ? 500 : \
				(hw_mpu9250_samplerate == HWSNAPDRAGONFLIGHT_MPU9250RATE_1000) ? 1000 : \
				pios_mpu9250_cfg.default_samplerate;
			PIOS_MPU9250_SetSampleRate(mpu9250_samplerate);
#endif /* PIOS_INCLUDE_MPU9250_SPI */
		}
		break;
	}

#if defined(PIOS_INCLUDE_ADC)
	PIOS_INTERNAL_ADC_Init(&internal_adc_id, &pios_adc_cfg);

	PIOS_ADC_Init(&pios_internal_adc_id, &pios_internal_adc_driver, internal_adc_id);
#endif

#if defined(PIOS_INCLUDE_FLASH_JEDEC)
	if ( PIOS_STREAMFS_Init(&streamfs_id, &streamfs_settings, FLASH_PARTITION_LABEL_LOG) != 0)
		panic(8);

	const uint32_t LOG_BUF_LEN = 256;
	uint8_t *log_rx_buffer = PIOS_malloc(LOG_BUF_LEN);
	uint8_t *log_tx_buffer = PIOS_malloc(LOG_BUF_LEN);
	if (PIOS_COM_Init(&pios_com_spiflash_logging_id, &pios_streamfs_com_driver, streamfs_id,
	                  log_rx_buffer, LOG_BUF_LEN, log_tx_buffer, LOG_BUF_LEN) != 0)
		panic(9);
#endif /* PIOS_INCLUDE_FLASH_JEDEC */

}

/**
 * @}
 */
