Tested on ROS2(Humble) with SocketCAN

Use along with https://github.com/BerenChang/vesc_msgs

## Control Mode
- In ROS, there are three regular control modes provided:
  - ThrottleBoard control,
  - Current Control,
  - Speed Control.

- Zero Turn mode is provided for additional maneuverability.
- Emergency Stop mode triggers a fault FAULT_CODE_CANBUS_EMERGENCY_STOP and cut the motor current right away.
- You can send other CAN commands to VESC as well. They can be found in datatype.h.

## ID Frame Format
We use 29-bit extended IDs for CANFD communication. Both VESC ID and the command ID are embedded in the extended ID of a CAN ID frame as:

| **B28 - B16** | **B15 - B8** | **B7 - B0** |
|-----------|----------|---------|
| Unused | Command ID | VESC ID |

where

- VESC ID is preset as 0x01 for master side, 0x02 for slave side.

- Command ID is set as:

| **Mode** | **Command ID** |
|------|-------------|
| ThrottleBoard control | CAN_SET_THROTTLE_BOARD | 
| Current Control | CAN_PACKET_SET_CURRENT | 
| Speed Control | CAN_SET_RPM_REL | 
| Zero Turn | CAN_ZERO_TURN |
| Emergency Stop | CAN_EMERGENCY_STOP |

## Data Frame Format

Each parameter occupies 4 bytes and is represented as a float32, scaled by 1e5 when packed into a CAN data frame.

For transmission purposes, all parameters are incremented by 1 in ROS to ensure they are positive. If you are implementing your own ROS node or sending commands directly to the firmware, be sure to add 1 to each parameter accordingly.

- ThrottleBoard control

| **Parameter(x)** | **Range** | **Description** |
|------|------|-------------|
| Throttle | -1 <= x <= 1 | -1 <= x < 0 for backwards, 0 < x <= 1 for forwards. |
| Board | -1 <= x <= 1 | -1 <= x < 0 for turning left, 0 < x <= 1 for turning right. |

- Current Control

| **Parameter** | **Range** | **Description** |
|------|------|-------------|
| current_rel0 | -1 <= x <= 1 | Relative current for master side. -1 <= x < 0 for negative current, 0 < x <= 1 for positive current. |
| current_rel1 | -1 <= x <= 1 | Relative current for slave side. -1 <= x < 0 for negative current, 0 < x <= 1 for positive current. |

- Speed Control

| **Parameter** | **Range** | **Description** |
|------|------|-------------|
| rpm_rel0 | -1 <= x <= 1 | Relative speed for master side. -1 <= x < 0 for negative speed, 0 < x <= 1 for positive speed. |
| rpm_rel1 | -1 <= x <= 1 | Relative speed for master side. -1 <= x < 0 for negative speed, 0 < x <= 1 for positive speed. |

- Zero Turn

| **Parameter** | **Range** | **Description** |
|------|------|-------------|
| Throttle | -1 <= x <= 1 | -1 <= x < 0 for backwards, 0 < x <= 1 for forwards. |
| Board | -1 <= x <= 1 | -1 <= x < 0 for turning left, 0 < x <= 1 for turning right. |

- Emergency Stop

  - No data frame needed.

## CANBUS Status Structure

- CAN status 1

| **Message** | **Unit** | **Description** |
|------|------|-------------|
| id | DNE | VESC ID. |
| rpm | rev/min | Revolutions per minute. |
| current | Amp | The current that goes into the motor. |
| duty | % | . |

- CAN status 2

| **Message** | **Unit** | **Description** |
|------|------|-------------|
| id | DNE | VESC ID. |
| amp_hours | Ah | Battery Amp Hours. |
| amp_hours_charged | Ah | Battery Amp Hours Charged. |

- CAN status 3

| **Message** | **Unit** | **Description** |
|------|------|-------------|
| id | DNE | VESC ID. |
| watt_hours | Wh | . |
| watt_hours_charged | Wh | . |

- CAN status 4

| **Message** | **Unit** | **Description** |
|------|------|-------------|
| id | DNE | VESC ID. |
| temp_fet | °C | Temperature of the MOSFET. |
| temp_motor | °C | Temperature of the the motor. |
| current_in | Amp | Current that goes into the motor. |
| pid_pos_now | . | . |

- CAN status 5

| **Message** | **Unit** | **Description** |
|------|------|-------------|
| id | DNE | VESC ID. |
| tacho_value | rev/min | . |
| v_in | Volt | The battery voltage. |


For more info, check: https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md
