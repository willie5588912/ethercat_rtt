#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

// Application parameters
#define FREQUENCY 500
#define PRIORITY 1

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domainInput = NULL;
static ec_domain_state_t domainInput_state = {};

static ec_domain_t *domainOutput = NULL;
static ec_domain_state_t domainOutput_state = {};

static ec_slave_config_t *sc_motor = NULL;
static ec_slave_config_state_t sc_motor_state = {};


/****************************************************************************/

// process data
static uint8_t *domainOutput_pd = NULL;
static uint8_t *domainInput_pd = NULL;

#define MotorSlavePos 0, 0
#define MBDHT2510BA1 0x0000066f, 0x525100a1

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
int enc_count = 10;

/*****************************************************************************/

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

/*****************************************************************************/

void check_domain_state(void)
{
  ec_domain_state_t ds;

  ecrt_domain_state(domainOutput, &ds);

  if (ds.working_counter != domainOutput_state.working_counter)
    printf("domainOutput: WC %u.\n", ds.working_counter);
  if (ds.wc_state != domainOutput_state.wc_state)
    printf("domainOutput: State %u.\n", ds.wc_state);

  domainOutput_state = ds;

  ecrt_domain_state(domainInput, &ds);

  if (ds.working_counter != domainInput_state.working_counter)
    printf("domainInput: WC %u.\n", ds.working_counter);
  if (ds.wc_state != domainInput_state.wc_state)
    printf("domainInput: State %u.\n", ds.wc_state);

  domainInput_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
  ec_master_state_t ms;

  ecrt_master_state(master, &ms);

  if (ms.slaves_responding != master_state.slaves_responding)
    printf("%u slave(s).\n", ms.slaves_responding);
  if (ms.al_states != master_state.al_states)
    printf("AL states: 0x%02X.\n", ms.al_states);
  if (ms.link_up != master_state.link_up)
    printf("Link is %s.\n", ms.link_up ? "up" : "down");

  master_state = ms;
}

/*****************************************************************************/

void check_slave_config_states(void)
{
  ec_slave_config_state_t s;
  ecrt_slave_config_state(sc_motor, &s);

  if (s.al_state != sc_motor_state.al_state)
    printf("Motor: State 0x%02X.\n", s.al_state);
  if (s.online != sc_motor_state.online)
    printf("Motor: %s.\n", s.online ? "online" : "offline");
  if (s.operational != sc_motor_state.operational)
    printf("Motor: %soperational.\n",s.operational ? "" : "Not ");

  sc_motor_state = s;

}

/****************************************************************************/
int igh_configure()
{
  // receive process data
  ecrt_master_receive(master);
  ecrt_domain_process(domainOutput);
  ecrt_domain_process(domainInput);

  curr_pos = EC_READ_S32(domainInput_pd + mbdh_actpos);
  printf("curr_pos = %d\n", curr_pos);
  //printf("6041h = %4.4x\n", EC_READ_U16(domainInput_pd + mbdh_statwd));


  static int state = -500;
  state++;
  switch(state)
  {
#if 1
    case 0:
      printf("change mode to csp\n");
      EC_WRITE_S8(domainOutput_pd + mbdh_modeop, 8);
    break;
#endif

    case 3:
      printf("shutdown\n");
      EC_WRITE_U16(domainOutput_pd + mbdh_cntlwd, 0x06);
    break;

    case 4:
      printf("switch on\n");
      EC_WRITE_U16(domainOutput_pd + mbdh_cntlwd, 0x07);
    break;

    case 5:
      printf("enable operation (should servo on now)\n");
      EC_WRITE_U16(domainOutput_pd + mbdh_cntlwd, 0xF);
    break;
  }

  // send process data
  ecrt_domain_queue(domainOutput);
  ecrt_domain_queue(domainInput);
  ecrt_master_send(master);

  return state;
}


void cyclic_task()
{
  // receive process data
  ecrt_master_receive(master);
  ecrt_domain_process(domainOutput);
  ecrt_domain_process(domainInput);

  curr_pos = EC_READ_S32(domainInput_pd + mbdh_actpos);
  printf("curr_pos = %d\n", curr_pos);

  EC_WRITE_S32(domainOutput_pd + mbdh_tarpos, curr_pos + enc_count);

  // send process data
  ecrt_domain_queue(domainOutput);
  ecrt_domain_queue(domainInput);
  ecrt_master_send(master);
}

#if 0
void cyclic_task()
{
  // receive process data
  ecrt_master_receive(master);
  ecrt_domain_process(domainOutput);
  ecrt_domain_process(domainInput);

  static int state = -500;
  static int enc_count = 100;
  //enc_count++;

  printf("6041h = %4.4x\n", EC_READ_U16(domainInput_pd + mbdh_statwd));
  state++;
  switch (state)
  {
#if 0
    case -2:
      printf("fault reset 1\n");
      EC_WRITE_U16(domainOutput_pd + mbdh_cntlwd, 0x00);
      printf("6041h = %4.4x\n", EC_READ_U16(domainInput_pd + mbdh_statwd));
    break;

    case -1:
      printf("fault reset 2\n");
      EC_WRITE_U16(domainOutput_pd + mbdh_cntlwd, 0x80);
      printf("6041h = %4.4x\n", EC_READ_U16(domainInput_pd + mbdh_statwd));
    break;
#endif

#if 1
    case 0:
      printf("change mode to csp\n");
      EC_WRITE_S8(domainOutput_pd + mbdh_modeop, 8);
      printf("6041h = %4.4x\n", EC_READ_U16(domainInput_pd + mbdh_statwd));
      //EC_WRITE_U32(domainOutput_pd + asdaa2_provel, 9000000);
    break;
#endif

    case 3:
      printf("shutdown\n");
      EC_WRITE_U16(domainOutput_pd + mbdh_cntlwd, 0x06);
      //printf("6041h = %4.4x\n", EC_READ_U16(domainInput_pd + mbdh_statwd));
    break;

    case 4:
      printf("switch on\n");
      EC_WRITE_U16(domainOutput_pd + mbdh_cntlwd, 0x07);
      //printf("6041h = %4.4x\n", EC_READ_U16(domainInput_pd + mbdh_statwd));
    break;

    case 1000:
      printf("enable operation (should servo on now)\n");
      EC_WRITE_U16(domainOutput_pd + mbdh_cntlwd, 0xF);
      //printf("6041h = %4.4x\n", EC_READ_U16(domainInput_pd + mbdh_statwd));
    break;

  }

  EC_WRITE_S32(domainOutput_pd + mbdh_tarpos, curr_pos + enc_count);

  // send process data
  ecrt_domain_queue(domainOutput);
  ecrt_domain_queue(domainInput);
  ecrt_master_send(master);
}
#endif

/****************************************************************************/
int main(int argc, char **argv)
{
  // Requests an EtherCAT master for realtime operation.
  master = ecrt_request_master(0); // Index of the master to request.
  if (!master)
    return -1;

  // Creates a new process data domain
  domainOutput = ecrt_master_create_domain(master);
  if (!domainOutput)
    return -1;
  domainInput = ecrt_master_create_domain(master);
  if (!domainInput)
    return -1;

  // Obtains a slave configuration
  //if (!(sc_motor = ecrt_master_slave_config(master, MotorSlavePos, Delta_ASDAA2))) 
  if (!(sc_motor = ecrt_master_slave_config(master, MotorSlavePos, MBDHT2510BA1))) 
  {
    fprintf(stderr, "Failed to get slave configuration.\n");
    return -1;
  }

  // Configuring PDOs
  printf("Configuring PDOs...\n");
  if (ecrt_slave_config_pdos(sc_motor, EC_END, mbdh_syncs))
  {
    fprintf(stderr, "Failed to configure PDOs.\n");
    return -1;
  }

  if (ecrt_domain_reg_pdo_entry_list(domainOutput, domainOutput_regs)) 
  {
    fprintf(stderr, "PDO entry registration failed!\n");
    return -1;
   }

  if (ecrt_domain_reg_pdo_entry_list(domainInput, domainInput_regs)) 
  {
    fprintf(stderr, "PDO entry registration failed!\n");
    return -1;
  }

  printf("Activating master...\n");
  if (ecrt_master_activate(master))
    return -1;

  if (!(domainOutput_pd = ecrt_domain_data(domainOutput))) 
  {
    return -1;
  }

  if (!(domainInput_pd = ecrt_domain_data(domainInput))) 
  {
    return -1;
  }
  
  //printf("wait for 5 seconds...\n");
  //sleep(5);

  printf("\n===============================\n");
  printf("Started.\n");

#if 1
  while (igh_configure() < 5) 
    usleep(1000);

  while(1)
  {
    usleep(1000);
    cyclic_task();
  }

#endif

  //stop_slave();

  return 0;
}
