package org.firstinspires.ftc.teamcode.MainBot.teleop;

public class CrossCommunicator {
    public static double mainBotThrottleFactor = 1.0;

    public static class Drive {
        public static final String MOTOR_FRONT_LEFT = "front_left";
        public static final String MOTOR_FRONT_RIGHT = "front_right";
        public static final String MOTOR_BACK_LEFT = "back_left";
        public static final String MOTOR_BACK_RIGHT = "back_right";
    }

    public static class Elevator {
        public static final String MOTOR = "elevator";
        public static final String SERVO = "elevator_retainer";

        public static void setIsExtended(boolean val) {
            mainBotThrottleFactor = val ? 0.25 : 1.0;
        }
    }

    public static class Feeder {
        public static final String SERVO = "ball_holder";
    }

    public static class Intake {
        public static final String MOTOR = "intake";
    }

    public static class Pinball {
        public static final String MOTOR = "pinball_motor";
        public static final String SERVO_FLAG = "semaphore";
        public static final String SERVO_CLAMP = "pinball_servo";
        public static final String SENSOR_TOUCH = "pinball_touch";
    }
}
