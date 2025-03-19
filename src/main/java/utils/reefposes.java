package utils;
import edu.wpi.first.math.util.Units;
import java.util.Hashtable;

import frc.robot.Constants;

public class reefposes {

    private static double d = 0.831596;
    private static Hashtable<String, Double[]> redAlliance;
    private static Hashtable<String, Double[]> blueAlliance;

    //private static double alpha = 0.459502;

    //private static double beta = -0.2359;
    
    public reefposes() {
        redAlliance = new Hashtable<>();
        blueAlliance = new Hashtable<>();
        
        Double[][] calc = calculatedAprilTagPos(0.459502+0.06, -0.2359);
        redAlliance.put(
            "A", 
            calc[2]
        );
        redAlliance.put(
            "B", 
            calc[3]
        );
        redAlliance.put(
            "C", 
            calc[4]
        );
        redAlliance.put(
            "D", 
            calc[5]
        );
        redAlliance.put(
            "E", 
            calc[6]
        );
        redAlliance.put(
            "F", 
            calc[7]
        );
        redAlliance.put(
            "G", 
            calc[8]
        );
        redAlliance.put(
            "H", 
            calc[9]
        );
        redAlliance.put(
            "I", 
            calc[10]
        );
        redAlliance.put(
            "J", 
            calc[11]
        );
        redAlliance.put(
            "K", 
            calc[0]
        );
        redAlliance.put(
            "L", 
            calc[1]
        );
       
        blueAlliance.put(
            "A", 
            calc[6]
        );
        blueAlliance.put(
            "B", 
            calc[7]
        );
        blueAlliance.put(
            "C", 
            calc[8]
        );
        blueAlliance.put(
            "D", 
            calc[9]
        );
        blueAlliance.put(
            "E", 
            calc[10]
        );
        blueAlliance.put(
            "F", 
            calc[11]
        );
        blueAlliance.put(
            "G", 
            calc[0]
        );
        blueAlliance.put(
            "H", 
            calc[1]
        );
        blueAlliance.put(
            "I", 
            calc[2]
        );
        blueAlliance.put(
            "J", 
            calc[3]
        );
        blueAlliance.put(
            "K", 
            calc[4]
        );
        blueAlliance.put(
            "L", 
            calc[5]
        );
    }

    public Double[][] calculatedAprilTagPos(double additional, double sideshift) {
        Double[][] displacement = new Double[12][];
        for (int i = -1; i<5; i++) {
            displacement[2*(i+1)] = new Double[]{
                Math.cos(Math.PI/3*i)*(d+additional)-sideshift*Math.sin(Math.PI/3*i),
                Math.sin(Math.PI/3*i)*(d+additional)+sideshift*Math.cos(Math.PI/3*i), 
                Math.PI/2.+Math.PI/3.*i
            };
            displacement[2*(i+1)+1] = new Double[]{
                Math.cos(Math.PI/3*i)*(d+additional)-(sideshift+Units.inchesToMeters(12.94))*Math.sin(Math.PI/3*i),
                Math.sin(Math.PI/3*i)*(d+additional)+(sideshift+Units.inchesToMeters(12.94))*Math.cos(Math.PI/3*i),
                Math.PI/2.+Math.PI/3.*i
            };
        }
        return displacement;
    }

    public Double[] getArrayfromKey(String key, Boolean left) {
        if (left) {
            return redAlliance.get(key);
        } else {
            return blueAlliance.get(key);
        }
    }

    // public Double[][] leftDisplacement(double shift) {
    //     Double[][] displacement = new Double[6][];
    //     for (int i = 0; i<5; i++) {
    //         displacement[i] = new Double[]{
    //             -shift*Math.sin(Math.PI/3*i), -shift*Math.cos(Math.PI/3*i)
    //         };
    //     }
    //     return displacement;
    // }

    // public Double[][] rightDisplacement(double shift) {
    //     Double[][] displacement = new Double[6][];
    //     for (int i = 0; i<5; i++) {
    //         displacement[i] = new Double[]{
    //             -(shift+Units.inchesToMeters(12.94))*Math.sin(Math.PI/3*i), -(shift+Units.inchesToMeters(12.94))*Math.cos(Math.PI/3*i)
    //         };
    //     }
    //     return displacement;
    // }

}