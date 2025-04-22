package utils;

public class ConvertAprilTag {
    // empty constructor
    public ConvertAprilTag() {}

    //
    public static int getTag(Reef.Location loc, boolean isRedAlliance) {
        if (isRedAlliance) {
        switch (loc) {
            case A,B:
            return 7;

            case C,D:
            return 8;

            case E,F:
            return 9;

            case G,H:
            return 10;

            case I,J:
            return 11;

            case K,L:
            return 6;
            
        }
    } else {
        switch (loc) {
            case A,B:
            return 18;

            case C,D:
            return 17;

            case E,F:
            return 22;

            case G,H:
            return 21;

            case I,J:
            return 20;

            case K,L:
            return 19;
        }
        }
        
        return 0;
    }

    // returns if the the robot is on the left side
    public static boolean leftSide(Reef.Location loc) {
        switch (loc) {
            case A,C,E,G,I,K:
                return true;
        
            default:
                return false;
        }
    }
}
