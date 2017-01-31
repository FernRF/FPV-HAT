#include <stdlib.h>
#include "si4463.h"

#include "radio_config_2fsk_rx.h"
#include "packet_config.h"


size_t packet_size_2fsk_rx()
{
    return PACKET_SIZE;
}

int load_2fsk_config_rx(si4463_t si)
{
    U8 radio_config[] = RADIO_CONFIGURATION_DATA_ARRAY;

    int cmds = SI4463_cfg_load(si, radio_config);

    return cmds;
}
