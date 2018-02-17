package org.firstinspires.ftc.teamcode.MainBot.teleop;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by gregory.ling on 9/27/17.
 */

public class CrossCommunicator {
    public static class Drive {                                    // FL––G––FR
        public static final String FRONT_LEFT = "front left";      // |U+   -R|
        public static final String BACK_RIGHT = "back right";      // J       |
        public static final String BACK_LEFT = "back left";        // |L+   -D|
        public static final String FRONT_RIGHT = "front right";    // BL-----BR
    }

    public static class Jewel {
        public static final String MOTOR = "jewel arm";
    }

    public static class Glyph {
        public static final String ELEV = "glyph elevator";
        public static final String GRAB_UPPER = "grab_upper";
        public static final String GRAB_LOWER = "grab_lower";
        public static final String VSD = "vsd";
    }

    public static class State {
        public static boolean justChanged = false;
        public static ElapsedTime time = new ElapsedTime();
        public static int yState = 3;
        public static void yIncrease() {yState = yState + 1 > 3? 0 : yState + 1;}
        public static void yDecrease() {yState = yState - 1 < 0? 3 : yState - 1;}
        public static boolean yIncreased = false;
        public static boolean yDecreased = false;
    }

}
