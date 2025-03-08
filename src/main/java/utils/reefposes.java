package utils;
import edu.wpi.first.math.util.Units;

import frc.robot.Constants;

public class reefposes {
    
    public static double[][] a;

    public reefposes() {
        a = centralEdges(14.32, 3.88, -90);
        System.out.println(a);
        printArray(displacementAddition(a));
    }

    public double[] center(double x, double y) {
        return new double[]{x-Constants.reefLength/2, y-Constants.reefLevelDistance/2};  
    }  
    
    public double[][] centralEdges(double initialX, double initialY, double initialT) {
        
        double[] centerpoint = center(initialX,initialY);
        double[][] centralEdges = new double[6][];

        for (int i=0; i<6; i++) {
            centralEdges[i] = new double[]{
                centerpoint[0]+Math.cos(Math.PI*i/3)*Constants.reefLength/2, //xpos
                centerpoint[1]+Math.sin(Math.PI*i/3)*Constants.reefLevelDistance/2, //ypos
                initialT+60*i, //angle
            };
        }

        return centralEdges;
    }

    public double[][] displacementAddition(double[][] centerEdges) {
        double[][] center = centerEdges;

        double[][] poses = new double[12][];

        for (int i=0; i<6; i+=1) {
            poses[2*i] = center[i];
            poses[2*i+1] = center[i];
        }

        
        //poses[0][1] = Constants.reefLevelDistance/2;

        poses[0][1] = poses[0][1] - Constants.reefLevelDistance/2;

        poses[1][1] = poses[0][1]+ Constants.reefLevelDistance/2;

        poses[2][0] += Math.cos(Math.PI/3)*Constants.reefLevelDistance/2;
        poses[2][1] -= Math.sin(Math.PI/3)*Constants.reefLevelDistance/2;

        poses[3][0] -= Math.cos(Math.PI/3)*Constants.reefLevelDistance/2;
        poses[3][1] += Math.sin(Math.PI/3)*Constants.reefLevelDistance/2;

        poses[4][0] += Math.cos(Math.PI/3)*Constants.reefLevelDistance/2;
        poses[4][1] += Math.sin(Math.PI/3)*Constants.reefLevelDistance/2;

        poses[5][0] -= Math.cos(Math.PI/3)*Constants.reefLevelDistance/2;
        poses[5][1] -= Math.sin(Math.PI/3)*Constants.reefLevelDistance/2;

        poses[6][1] += Constants.reefLevelDistance/2;

        poses[7][1] -= Constants.reefLevelDistance/2;

        poses[8][0] += Math.cos(Math.PI/3)*Constants.reefLevelDistance/2;
        poses[8][1] -= Math.sin(Math.PI/3)*Constants.reefLevelDistance/2;

        poses[9][0] -= Math.cos(Math.PI/3)*Constants.reefLevelDistance/2;
        poses[9][1] += Math.sin(Math.PI/3)*Constants.reefLevelDistance/2;

        poses[10][0] -= Math.cos(Math.PI/3)*Constants.reefLevelDistance/2;
        poses[10][1] -= Math.sin(Math.PI/3)*Constants.reefLevelDistance/2;

        poses[11][0] += Math.cos(Math.PI/3)*Constants.reefLevelDistance/2;
        poses[11][1] += Math.sin(Math.PI/3)*Constants.reefLevelDistance/2;

        return poses;

    }

    public void printArray(double[][] array) {
        for (double[] row: array) {
            for (double value:row) {
                System.out.println(value);
            }
            System.out.println();
        }
    }



}
