# Motor control in STM32  

[RoboMaster M3508](https://github.com/RoboGrinder-ECE/Motor_Control/blob/master/Document/RoboMaster%20M3508%20P19%20Brushless%20DC%20Gear%20Motor%20V1.0.pdf)
 motor and 
 [C620 ESC](https://github.com/RoboGrinder-ECE/Motor_Control/blob/master/Document/RoboMaster%20C620%20Brushless%20DC%20Motor%20Speed%20Controller%20V1.01.pdf)
 is used for our robot. 

## Signal modes in C620 ESC
C620 ESC supports two signal modes.
1. 20-500Hz PWM (We rarely use the PWM to control the C620 since there is no feedback from the motor.)
2. CAN communication

## CAN Commnication Protocol
**The bitrate of the CAN bus is 1 Mbps**  
Read though the
[C620 ESC](https://github.com/RoboGrinder-ECE/Motor_Control/blob/master/Document/RoboMaster%20C620%20Brushless%20DC%20Motor%20Speed%20Controller%20V1.01.pdf)
Documentation CAN Communication Protocol section from page 14 to page 17

## Example: Control the current output for the motor with id = 1
1. Prepare the CAN header and data buffer
```
txHeader.StdId = 0x200;
txHeader.RTR   = CAN_RTR_DATA;
txHeader.IDE   = CAN_ID_STD;
txHeader.DLC   = 8;
txHeader.TransmitGlobalTime = DISABLE;
txData[0] = 0;
txData[1] = 0;
txData[2] = 0;
txData[3] = 0;
txData[4] = 0;
txData[5] = 0;
txData[6] = 0;
txData[7] = 0;
```
2. Set the current for the motor with id = 1
```
int16_t current = 500;
txData[0] = current >> 8;
txData[1] = current;
```
**Why use int16_t instead of int?**  
**What does current >> 8 do?**
3. Send the current command
```
if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox) != HAL_OK) {
  Error_Handler();
}
```

## Example: Receive and decode the motor feedback message
1. Prepare the CAN header and data buffer to receive the message
```
CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];
```
2. Enable the CAN receive interrupt
[**An Introduction to Interrupts**](https://www.youtube.com/watch?v=jMnuQMYR3Ro)
```
if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
  Error_Handler();
}
```
3. Prepare a struct to store motor data
**Why we need a struct?**
```
typedef struct
{
  uint16_t ecd;
  uint16_t last_ecd;
  
  int16_t  speed_rpm;
  int16_t  given_current;

  int32_t  round_cnt;
  int32_t  total_ecd;
  int32_t  total_angle;
  
  uint16_t offset_ecd;
  uint32_t msg_cnt;
	
	int32_t ecd_raw_rate;
} moto_measure_t;
```
4. Prepare a function to get init offset of the motor
```
void get_moto_offset(moto_measure_t* ptr, uint8_t rxData[])
{
    ptr->ecd        = (uint16_t)(rxData[0] << 8 | rxData[1]);
    ptr->offset_ecd = ptr->ecd;
}

```
5. Prepare a function to update the motor info
```
void encoder_data_handler(moto_measure_t* ptr, uint8_t rxData[])
{
  ptr->last_ecd = ptr->ecd;
  ptr->ecd      = (uint16_t)(rxData[0] << 8 | rxData[1]);
  
  if (ptr->ecd - ptr->last_ecd > 4096)
  {
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
  else if (ptr->ecd - ptr->last_ecd < -4096)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  /* total angle, unit is degree */
  ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;
	ptr->speed_rpm     = ((int16_t)rxData[2] << 8 | rxData[3]);
  ptr->given_current = ((int16_t)rxData[4] << 8 | rxData[5]);
}
```
6. Prepare CAN rx callback function
```
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData);
	if (rxHeader.StdId == 0x201) {
		moto1.msg_cnt++ <= 50 ? get_moto_offset(&moto1, rxData) : encoder_data_handler(&moto1, rxData);
	}
}
```

## Things to be done before Saturaday workshop
Watch the PID tutorial video: [Link](https://www.youtube.com/watch?v=wkfEZmsQqiA&list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y)
