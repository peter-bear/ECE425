| Topic Name                        | Data Format                                                  | Description                              | From  |
| --------------------------------- | ------------------------------------------------------------ | ---------------------------------------- | ----- |
| walter/lidar_data                 | 10 20 30 40                                                  | front back left right                    | robot |
| walter/sonar_data                 | 30 25                                                        | leftSonar rightSonar                     | robot |
| walter/led_data                   | 0 0 0                                                        | RedLed  GreenLed YelloLed                | robot |
| walter/move_control               | MOVE_FORWARD / TURN_LEFT / TURN_RIGHT / MOVE_BACKWARD / STOP |                                          | GUI   |
| walter/encoder_data               | 300 200                                                      | leftEncoder rightEncoder                 | robot |
| walter/robot_path_plan_position   | 3 0 0 3                                                      | start position (x y) goal position (x y) | GUI   |
| walter/robot_path_plan            | 4 0 ; 4 1; 3 1; 2 1; 2 2; 2 3; 1 3 ; 0 3; 0 4                | planned path positions (x y)             | robot |
| walter/robot_position             | 4 0                                                          | robot position (x y)                     | robot |
| walter/matrix_map                 | 0 99 99 0; 0 0 0 0; 0 99 99 0; 0 99 0 0;                     | map matrix data                          | robot |
| walter/grid_localization_response | 4 0 ; 4 1; 3 1; 2 1; 2 2; 2 3; 1 3 ; 0 3; 0 4                | all possible positions                   | robot |
| walter/grid_localization_command  | START / STOP                                                 | start grid localization or stop          | GUI   |

