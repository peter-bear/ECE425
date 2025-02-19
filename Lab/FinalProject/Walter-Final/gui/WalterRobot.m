classdef WalterRobot < hgsetget
    properties
        mqClient
        lidarData
        sonarData
        ledData
        encoderData
        matrixMapData
        robotPosition
        gui
    end

    properties (Constant = true)
        lidarDataTopic = "walter/lidar_data";
        sonarDataTopic = "walter/sonar_data";
        ledDataTopic = "walter/led_data";
        encoderDataTopic = "walter/encoder_data";
        moveControlTopic = "walter/move_control";
        matrixDataTopic = "walter/matrix_map";
        robotPositionTopic = "walter/robot_position";
        robotPathPlanTopic = "walter/robot_path_plan";
        robotPathPlanPositionTopic = "walter/robot_path_plan_position";
        gridLocalizationResponseTopic = "walter/grid_localization_response";
        gridLocalizationCommandTopic = "walter/grid_localization_command";
       
        moveForward = "MOVE_FORWARD";
        moveBackward = "MOVE_BACKWARD";
        turnLeft = "TURN_LEFT";
        turnRight = "TURN_RIGHT"
        stopMove = "STOP";

        startLocalization = "START";
        endLocalization = "STOP";

    end

    methods
        function obj = WalterRobot()
           
        end

        function sendForwardControl(obj)
            obj.mqClient.write(obj.moveControlTopic, obj.moveForward);
        end

        function sendBackwardControl(obj)
            obj.mqClient.write(obj.moveControlTopic, obj.moveBackward);
        end

        function sendTurnLeftControl(obj)
            obj.mqClient.write(obj.moveControlTopic, obj.turnLeft);
        end

        function sendTurnRightControl(obj)
            obj.mqClient.write(obj.moveControlTopic, obj.turnRight);
        end

        function sendMoveStopControl(obj)
            obj.mqClient.write(obj.moveControlTopic, obj.stopMove);
        end
        
        function sendPathPlanPosition(obj, positions)
            obj.mqClient.write(obj.robotPathPlanPositionTopic, positions);
        end

        function sendLocalizationCommand(obj, command)
            obj.mqClient.write(obj.gridLocalizationCommandTopic, command);
        end

    end
end