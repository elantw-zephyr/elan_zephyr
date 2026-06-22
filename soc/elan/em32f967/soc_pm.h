#ifndef SOC_ELAN_EM32F967_PM_H
#define SOC_ELAN_EM32F967_PM_H

/* Power Management Configuration */
#define EM32_DEEP_SLEEP_TARGET_UA   140  /* Target power: 140 μA */
#define EM32_WAKE_LATENCY_MS        300  /* Resume time: ~300ms */

/* External Function Declarations */
int em32f967_set_deep_sleep_config(int sensor_mode, int rtc_enabled);
void em32f967_enter_deep_sleep(void);
void em32f967_exit_deep_sleep(void);
int em32f967_enter_standby1(int rtc_enabled);
int em32f967_validate_deep_sleep_config(void);

/* Startup and Configuration Functions */
void soc_setup_startup_configuration(void);

/* GPIO Wake-Up Functions */
void soc_setup_external_wakeup(int edge_type);
void soc_disable_external_wakeup(void);
void soc_configure_gpio_pulldown(void);
int soc_get_power_state(void);
void soc_enable_usb_remote_wakeup(void);
void soc_configure_clock_gates(int mode);
int soc_prepare_for_deep_sleep(void);
void soc_handle_wakeup(void);
uint32_t soc_read_pa15_status(void);

#endif
