# Base Connections
| Connection A | Connection B |
| :---: | :---: |
| Control Hub Battery Connection (F) | I/O Power Switch (M) |
| Control Hub Battery Connection (M) | Expansion Hub Battery Connection (F) |
| Control Hub RS485 Connection Port (F) | Expansion Hub RS485 Connection Port (F) |


# Drivetrain Motors
## Control Hub
| Motor Slot | Name | Motor Type | Motor RPM |
| :---: | :---: | :---: | :---: |
| 0 | FL | GoBilda 5203 Motor | 1,150 RPM |
| 1 | BL | GoBilda 5203 Motor | 1,150 RPM |
| 2 | IN | GoBilda 5203 Motor | 1,150 RPM |
## Expansion Hub
| Motor Slot | Name | Motor Type | Motor RPM |
| :---: | :---: | :---: | :---: |
| 0 | FR | GoBilda 5203 Motor | 1,150 RPM |
| 1 | BR | GoBilda 5203 Motor | 1,150 RPM |

# Odometry
## Control Hub
| I<sup>2</sup>C Slot | Name | I<sup>2</sup>C Connection | Internal/External |
| :---: | :---: | :---: | :---: |
| 0 | imu | Control Hub IMU | Internal | 
| 1 | pinpoint | GoBilda Pinpoint Odometry Computer | External |
