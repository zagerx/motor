/**
 * @brief Set motor operating mode
 * @param mode Requested mode (MOTOR_CMD_SET_SPEED_MODE/MOTOR_CMD_SET_TORQUE_MODE)
 *
 * Iterates through all configured motors and sets their command mode.
 * Skips motors that aren't ready.
 */
 void motor_cmd_set(int16_t cmd,void *pdata,int8_t datalen)
 {
     /* Get motor devices from device tree */
     const struct device *motors[] = {
       #if DT_NODE_HAS_STATUS(DT_NODELABEL(motor0), okay)
               DEVICE_DT_GET(DT_NODELABEL(motor0)),
       #endif
       #if DT_NODE_HAS_STATUS(DT_NODELABEL(motor1), okay)
               DEVICE_DT_GET(DT_NODELABEL(motor1))
       #endif
       };
   
       const struct device *motor;
       struct motor_data *data;
        fsm_cb_t *fsm;
       const struct motor_config *cfg;
       const struct device *foc_dev;
 
       /* Process each motor */
       for(uint8_t i = 0;i < ARRAY_SIZE(motors);i++)
       {
         if (!device_is_ready(motors[i])) 
         {
           LOG_ERR("Motor %d not ready", i);
           continue;
         }
         
         motor = motors[i];
         data = motor->data;
         cfg = motor->config;
         fsm = cfg->fsm;
         foc_dev = cfg->foc_dev;
         switch (cmd) {
           case  MOTOR_CMD_SET_SPEED_MODE:
             data->cmd = MOTOR_CMD_SET_SPEED_MODE;
             break;
           case MOTOR_CMD_SET_TORQUE_MODE:
             data->cmd = MOTOR_CMD_SET_TORQUE_MODE;
             break;
           case MOTOR_CMD_SET_POSTION_MODE:
             data->cmd = MOTOR_CMD_SET_POSTION_MODE;
             break;
           case MOTOR_CMD_SET_ENABLE:
             statemachine_setsig(fsm,MOTOR_CMD_SET_ENABLE);
             break;
           case MOTOR_CMD_SET_DISABLE:
             statemachine_setsig(fsm,MOTOR_CMD_SET_DISABLE);
             break;
           case MOTOR_CMD_SET_SPEED:
             {
               if(data->mode == MOTOR_MODE_SPEED){
                 foc_write_data(foc_dev,FOC_PARAM_SPEED_REF,(float *)pdata);                
               }else if(data->mode == MOTOR_MODE_TORQUE){
                 foc_write_data(foc_dev,FOC_PARAM_DQ_REF,(float *)pdata);
               }else if(data->mode == MOTOR_MODE_POSI){
                 foc_write_data(foc_dev, FOC_PARAM_POSI_REF,(float *)pdata);
               }
               statemachine_setsig(fsm,MOTOR_CMD_SET_SPEED);
             }
             break;
           case MOTOR_CMD_SET_PIDPARAM:
             if(data->statue != MOTOR_STATE_IDLE)
             {
               break;
             }
             static float buf[4]; 
             float *param = (float *)pdata;
             buf[0] = param[0];buf[1] = param[1];
             statemachine_setsig(fsm,MOTOR_CMD_SET_PIDPARAM);
             fsm->p2 = (void *)buf;
             fsm->p2_len = datalen;
             break;
           default:
             break;
           }
       }     
 }

/**
 * @brief Main motor control task
 * @param obj Unused parameter
 *
 * Handles:
 * 1. Mode switching between control states
 * 2. FSM dispatching
 * Runs periodically to update motor control.
 */
void motor_task(void *obj)
{
    /* Get motor devices from device tree */
    const struct device *motors[] = {
    #if DT_NODE_HAS_STATUS(DT_NODELABEL(motor0), okay)
            DEVICE_DT_GET(DT_NODELABEL(motor0)),
    #endif
    #if DT_NODE_HAS_STATUS(DT_NODELABEL(motor1), okay)
            DEVICE_DT_GET(DT_NODELABEL(motor1))
    #endif
    };

    const struct device *motor;
    struct motor_data *data;

    /* Process each motor */
    for(uint8_t i = 0;i < ARRAY_SIZE(motors);i++)
    {
        if (!device_is_ready(motors[i])) 
        {
            LOG_ERR("Motor %d not ready", i);
            continue;
        }

        motor = motors[i];
        const struct motor_config *cfg = motor->config;
        data = motor->data;

        /* Handle mode change requests */
        switch (data->cmd) {
            case MOTOR_CMD_SET_SPEED_MODE:
                if(data->mode != MOTOR_MODE_SPEED)
                {
                  TRAN_STATE(cfg->fsm, motor_speed_control_mode);
                }
            break;
            case MOTOR_CMD_SET_TORQUE_MODE:
                if(data->mode != MOTOR_MODE_TORQUE)
                {
                  TRAN_STATE(cfg->fsm, motor_torque_control_mode);
                }
            break;
            case MOTOR_CMD_SET_POSTION_MODE:
                if(data->mode != MOTOR_MODE_POSI)
                {
                  TRAN_STATE(cfg->fsm, motor_position_control_mode);
                }
            break;
            default:
            break;
        }
        /* Run state machine */
        DISPATCH_FSM(cfg->fsm);
    }
}

