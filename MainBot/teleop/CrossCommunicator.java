package org.firstinspires.ftc.teamcode.MainBot.teleop;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by gregory.ling on 9/27/17.
 */

public class CrossCommunicator {
    public static class Drive {                      // FL––G––FR
        public static String UP = "front left";      // |U     R|
        public static String DOWN = "back right";    // J       |
        public static String LEFT = "back left";     // |L     D|
        public static String RIGHT = "front right";  // BL-----BR
    }

    public static class Jewel {
        public static String MOTOR = "jewel arm";
    }

    public static class Glyph {
        public static String ELEV = "glyph elevator";
        public static String GRAB = "grabber";
    }

    public static class State {
        public static boolean homeward = false;
        public static boolean justChanged = false;
        public static int curCol = 0;
        public static ElapsedTime time = new ElapsedTime();
    }

}
