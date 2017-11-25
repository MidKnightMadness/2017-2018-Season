package org.firstinspires.ftc.teamcode.MainBot.teleop;

/**
 * Created by gregory.ling on 9/27/17.
 */

public class CrossCommunicator {
    public static class Drive {                      // FL––G––FR
        public static String UP = "front left";      // |U     R|
        public static String DOWN = "back right";    // J       |
        public static String LEFT = "back left";     // |L     D|
        public static String RIGHT = "front right";  // BL–-––-BR
    }

    public static class Jewel {
        public static String MOTOR = "jewel arm";
    }

    public static class Glyph {
        public static String MOTOR = "glyph elevator";
        public static String SERVO = "grabber";
    }

    public static class State {
        public static boolean homeward = false;
        public static boolean justChanged = false;
    }

}
