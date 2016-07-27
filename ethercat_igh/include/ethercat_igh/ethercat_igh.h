#ifndef ETHERCAT_IGH
#define ETHERCAT_IGH

#include <stdbool.h>
#include "ecrt.h"

#define MotorSlavePos 0, 0
#define MBDHT2510BA1 0x0000066f, 0x525100a1

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domainInput = NULL;
static ec_domain_state_t domainInput_state = {};

static ec_domain_t *domainOutput = NULL;
static ec_domain_state_t domainOutput_state = {};

static ec_slave_config_t *sc_motor = NULL;
static ec_slave_config_state_t sc_motor_state = {};

// process data
static uint8_t *domainOutput_pd = NULL;
static uint8_t *domainInput_pd = NULL;

// offsets for PDO entries
static unsigned int mbdh_cntlwd;
static unsigned int mbdh_modeop;
static unsigned int mbdh_tarpos;
static unsigned int mbdh_tpbfnc;
static unsigned int mbdh_errcod;
static unsigned int mbdh_statwd;
static unsigned int mbdh_modedp;
static unsigned int mbdh_actpos;
static unsigned int mbdh_tpdsta;
static unsigned int mbdh_tpbpos;
static unsigned int mbdh_errval;
static unsigned int mbdh_digiin;

const static ec_pdo_entry_reg_t domainOutput_regs[] = {
 { MotorSlavePos, MBDHT2510BA1, 0x6040, 0, &mbdh_cntlwd },
 { MotorSlavePos, MBDHT2510BA1, 0x6060, 0, &mbdh_modeop },
 { MotorSlavePos, MBDHT2510BA1, 0x607A, 0, &mbdh_tarpos },
 { MotorSlavePos, MBDHT2510BA1, 0x60B8, 0, &mbdh_tpbfnc },
 {}
};

const static ec_pdo_entry_reg_t domainInput_regs[] = {
 { MotorSlavePos, MBDHT2510BA1, 0x603f, 0, &mbdh_errcod },
 { MotorSlavePos, MBDHT2510BA1, 0x6041, 0, &mbdh_statwd },
 { MotorSlavePos, MBDHT2510BA1, 0x6061, 0, &mbdh_modedp },
 { MotorSlavePos, MBDHT2510BA1, 0x6064, 0, &mbdh_actpos },
 { MotorSlavePos, MBDHT2510BA1, 0x60B9, 0, &mbdh_tpdsta },
 { MotorSlavePos, MBDHT2510BA1, 0x60BA, 0, &mbdh_tpbpos },
 { MotorSlavePos, MBDHT2510BA1, 0x60F4, 0, &mbdh_errval },
 { MotorSlavePos, MBDHT2510BA1, 0x60FD, 0, &mbdh_digiin },
 {}
};

static unsigned int counter = 0;
static unsigned int curr_pos = 0;
static unsigned int target_pos = 0;

static ec_pdo_entry_info_t mbdh_pdo_entries_output[] = {
 { 0x6040, 0x00, 16 },
 { 0x6060, 0x00, 8 },
 { 0x607A, 0x00, 32 },
 { 0x60B8, 0x00, 16 },
};

static ec_pdo_entry_info_t mbdh_pdo_entries_input[] = {
 { 0x603f, 0x00, 16 },
 { 0x6041, 0x00, 16 },
 { 0x6061, 0x00, 8 },
 { 0x6064, 0x00, 32 },
 { 0x60B9, 0x00, 16 },
 { 0x60BA, 0x00, 32 },
 { 0x60F4, 0x00, 32 },
 { 0x60FD, 0x00, 32 },
};

static ec_pdo_info_t mbdh_pdo_1600[] = {
 { 0x1600, 4, mbdh_pdo_entries_output },
};

static ec_pdo_info_t mbdh_pdo_1a00[] = {
 { 0x1A00, 8, mbdh_pdo_entries_input },
};

static ec_sync_info_t mbdh_syncs[] = {
 { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
 { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
 { 2, EC_DIR_OUTPUT, 1, mbdh_pdo_1600, EC_WD_DISABLE },
 { 3, EC_DIR_INPUT, 1, mbdh_pdo_1a00, EC_WD_DISABLE },
 { 0xff }
};

bool igh_configure();
int  igh_update(int);
void igh_cleanup();
int  ini_driver();
void check_domain_state();
void check_master_state();
void check_slave_config_states();

#endif
