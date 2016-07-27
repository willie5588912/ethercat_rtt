#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <ethercat_igh/ethercat_igh.h>

#define CHECK_BIT(var, pos) ((var) & (1 << (pos)))

bool igh_configure()
{
  // Requests an EtherCAT master for realtime operation.
  master = ecrt_request_master(0); // Index of the master to request.
  if (!master)
    return false;

  // Creates a new process data domain
  domainOutput = ecrt_master_create_domain(master);
  if (!domainOutput)
    return false;
  domainInput = ecrt_master_create_domain(master);
  if (!domainInput)
    return false;

  // Obtains a slave configuration
  //if (!(sc_motor = ecrt_master_slave_config(master, MotorSlavePos, Delta_ASDAA2))) 
  if (!(sc_motor = ecrt_master_slave_config(master, MotorSlavePos, MBDHT2510BA1))) 
  {
    fprintf(stderr, "Failed to get slave configuration.\n");
    return false;
  }

  // Configuring PDOs
  printf("Configuring PDOs...\n");
  if (ecrt_slave_config_pdos(sc_motor, EC_END, mbdh_syncs))
  {
    fprintf(stderr, "Failed to configure PDOs.\n");
    return false;
  }

  if (ecrt_domain_reg_pdo_entry_list(domainOutput, domainOutput_regs)) 
  {
    fprintf(stderr, "PDO entry registration failed!\n");
    return false;
   }

  if (ecrt_domain_reg_pdo_entry_list(domainInput, domainInput_regs)) 
  {
    fprintf(stderr, "PDO entry registration failed!\n");
    return false;
  }

  printf("Activating master...\n");
  if (ecrt_master_activate(master))
    return false;

  if (!(domainOutput_pd = ecrt_domain_data(domainOutput))) 
  {
    return false;
  }

  if (!(domainInput_pd = ecrt_domain_data(domainInput))) 
  {
    return false;
  }

  return true;
}


bool igh_start()
{
  int state = -500;
  while (state <= 5) 
  {
    ini_driver(state);
    state++;
    usleep(1000);
  }

  uint16_t statwd = EC_READ_U16(domainInput_pd + mbdh_statwd);
  printf("6041h = %4.4x\n",statwd); 
  if( CHECK_BIT(statwd, 0) && !CHECK_BIT(statwd, 1) &&
     !CHECK_BIT(statwd, 2) && !CHECK_BIT(statwd, 3) &&
      CHECK_BIT(statwd, 5) && !CHECK_BIT(statwd, 6))
  {
    printf("Now in CSP mode, ready to receive commands.\n");
    return true;
  }
  else
  {
    printf("Servo on fail.\n");
    return false;
  }
}

int igh_update(int enc_count)
{
  counter++;

  // receive process data
  ecrt_master_receive(master);
  ecrt_domain_process(domainOutput);
  ecrt_domain_process(domainInput);

  // periodically check the states and show the current pose
  //if(counter % 100 == 0)
  curr_pos = EC_READ_S32(domainInput_pd + mbdh_actpos);
  //printf("curr_pos = %d\n", curr_pos);

  // write target position
  target_pos += enc_count; 
  //printf("target_pos = %d\n", target_pos);
  EC_WRITE_S32(domainOutput_pd + mbdh_tarpos, target_pos);

  // send process data
  ecrt_domain_queue(domainOutput);
  ecrt_domain_queue(domainInput);
  ecrt_master_send(master);

  return curr_pos;
}

void igh_stop()
{
  //ecrt_master_deactivate(master);

  // receive process data
  ecrt_master_receive(master);
  ecrt_domain_process(domainOutput);
  ecrt_domain_process(domainInput);

  EC_WRITE_U16(domainOutput_pd + mbdh_cntlwd, 0x00);

  // send process data
  ecrt_domain_queue(domainOutput);
  ecrt_domain_queue(domainInput);
  ecrt_master_send(master);
}

void igh_cleanup() 
{
  ecrt_release_master(master);
}

int ini_driver(int state)
{
  // receive process data
  ecrt_master_receive(master);
  ecrt_domain_process(domainOutput);
  ecrt_domain_process(domainInput);

  curr_pos = EC_READ_S32(domainInput_pd + mbdh_actpos);
  //printf("curr_pos = %d\n", curr_pos);

  target_pos = EC_READ_S32(domainInput_pd + mbdh_actpos);
  //printf("target_pos = %d\n", target_pos);

  switch(state)
  {
    case -100:
      printf("fault reset\n");
      EC_WRITE_U16(domainOutput_pd + mbdh_cntlwd, 0x80);
    break;

    case 0:
      printf("change mode to csp\n");
      EC_WRITE_S8(domainOutput_pd + mbdh_modeop, 8);
    break;

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

void check_domain_state()
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

void check_master_state()
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

void check_slave_config_states()
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
