package utils;
public class Reef{

    public static volatile int pos = 3; // current pos to go to

    /**
     * Reef Levels. L2 - L3 are the reef stalks
     * 
     * L1 is not included here! it is outaken from the intake arm
     */
    public enum Level {
        L2,
        L3,
        L4
    }
    
    /**
     * Locations around the reef. They are in pairs. A would be left, B would be right. and this would continue
     */
    public enum Location{
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L
    }
}

