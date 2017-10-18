package org.firstinspires.ftc.teamcode.MainBot.teleop;

/**
 * Created by gregory.ling on 9/27/17.
 */

public class CrossCommunicator {
    public static class Drive {                      // FL––––FR
        public static String UP = "front left";      // |U   R|
        public static String DOWN = "back right";    // |     |
        public static String LEFT = "back left";     // |L   D|
        public static String RIGHT = "front right";  // BL––––BR
    }

    public static class Jewel {
        public static String MOTOR = "jewel arm";
    }

}
