# mk10-vcu


IMPORTANT PARAMETERS IN THE TOP OF MAIN.C

/* options:
 * DRIVE : drive the car
 * UNWELD_AIRS : flicker both airs and precharge relay one by one
 * CALIBRATE_PEDALS : calibrate pedals. put apps1_as_percent, apps1Value, apps2_as_percent, apps_plausible, readsPer100ms, etc.
 * CAN_TEST : test can :skull:
 * DEBUG : idfk you choose
*/
define VCUMODE DRIVE


/**
 * one button RTD mode; press just the RTD button to both precharge and RTD when ready
 */
define ONE_BUTTON_RTD_MODE_ENABLE 1