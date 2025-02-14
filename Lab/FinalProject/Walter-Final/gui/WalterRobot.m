classdef WalterRobot < hgsetget
    properties
        mqClient
        lidarData
        sonarData
        ledData
        encoderData
        gui
    end

    properties (Constant = true)
        lidarDataTopic = "walter/lidar_data";
        sonarDataTopic = "walter/sonar_data";
        ledDataTopic = "walter/led_data";
        encoderDataTopic = "walter/encoder_data";
        moveControlTopic = "walter/move_control";
        matrixDataTopic = "walter/map_data";

        moveForward = "MOVE_FORWARD";
        moveBackward = "MOVE_BACKWARD";
        turnLeft = "TURN_LEFT";
        turnRight = "TURN_RIGHT"
        stopMove = "STOP";

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
        
    end
end