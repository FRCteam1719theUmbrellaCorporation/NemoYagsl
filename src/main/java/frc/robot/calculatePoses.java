package frc.robot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveDrive;
import java.util.function.BooleanSupplier;
import swervelib.imu.Pigeon2Swerve;
import frc.robot.subsystems.LimeLightExtra;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class calculatePoses{
    
    public calculatePoses() {
        System.out.println(centralEdges(14.32, 3.88, -90));
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
                //x displacement
                //y displacement
            };
        }

        return centralEdges;
    }

    
    

}
