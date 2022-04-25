float64 pack_voltage
float64 charge_current
float64 discharge_current
int32 state_of_charge
float64 remaining_time_to_full_charge
int32 remaining_capacity
int32 state_of_health
int32 DISCHARGING=1 #bit 0
int32 CHARGING=2    #bit 1
int32 OV=4         #bit 2 Over voltage
int32 UV=8          #bit 3 Under voltage
int32 COC=16         #bit 4 Charge over current
int32 DOC=32         #bit 5 Discharge over current
int32 DOT=64         #bit 6 Discharge over temperature
int32 DUT=128         #bit 7 Discharge under temperature
int32  SC=512         #bit 9
int32 COT=1024         #bit 10 Charge over temperature
int32 CUT=2048         #bit 11 Charge under temperature
int32 FW_STATUS_MSK=2031616 # to get Battery_Firmware_Status  do the following:
int32 FW_STATUS_SHIFT=16    # batt_fw_stat=(status_flags & FW_STATUS_MSK)>>FW_STATUS_SHIFT
int32 FW_UPD_OK=0                  #Battery firmware update finished OK.
int32 FW_UPD_RUNNING=1             #Battery firmware update running.
int32 FW_UPD_FAILED_BOOT=2         #Battery firmware update failed in Bootloader (Robot must not drive)
int32 FW_UPD_FAILED_APP=3          #Battery firmware update failed updating the application (Robot can drive with old FW)
int32 FW_UPD_FAILED_PARAM=4        #Battery firmware update failed uploading parameters (Robot can drive with old fw and parameters.)
int32 FW_STATUS_LOW_BATT=5         #Battery firmware update skipped battery too low (Robot can drive with old parameters.)
int32 FW_STATUS_FILE_CORRUPTED=6   #Battery firmware file corrupted (Robot can drive with old parameters.)
int32 FW_STATUS_CURRENT_TO_HIGH=7  #Battery firmware file corrupted (Robot can drive with old parameters.)
int32 FW_STATUS_NO_CAN=8           #Battery firmware update skipped no CAN communication (Robot can drive with old fw and parameters.)
int32 FW_BATTERY_IMBALANCE_HIGH=9  #Battery firmware update is enforced and the battery will be shut off by the new firmware
int32 status_flags
int32 temperature
uint32[] cell_voltage # In Mk2 robots and above the BMS provides data for 8 battery cells. 2Gen robots have BMS for 13 battery cells

# Exteded diagnosticts for BMZ battery
uint32 bmz_flag # Flag for enabling extended diagnosticts
float64 full_voltage
int32 full_capacity
int32 temperature2
int32 temperature3
int32 cycle_count
int32 dsg_overcurrent_counter
int32 chg_overcurrent_counter
int32 hw_major
int32 hw_minor
int32 fw_major
int32 fw_minor
int32 fw_patch
int32 rec_fw_major
int32 rec_fw_minor
int32 bl_major
int32 bl_minor
uint32 status_enabled
uint32 status_current_limitation
uint32 status_switch_off_warn1
uint32 status_switch_off_warn2
uint32 status_fully_discharged
uint32 status_nearly_discharged
uint32 status_chargefet_on
uint32 status_dischargefet_on
uint32 status_discharging
uint32 status_fully_charged
uint32 status_charging
uint32 status_temp_charging_err
uint32 status_cell_over_voltage
uint32 status_cell_under_voltage
uint32 status_charge_over_current
uint32 status_shortcircuit
uint32 status_discharge_over_current
uint32 status_temp_discharging_err
uint32 status_charger_detected

float64 last_battery_msg_time
