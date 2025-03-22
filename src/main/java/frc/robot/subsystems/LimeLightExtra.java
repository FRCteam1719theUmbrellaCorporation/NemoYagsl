package frc.robot.subsystems;


import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.imu.Pigeon2Swerve;

public class LimeLightExtra {

    // old limelight names
    public final static String backCam = "limelight-back";
    public final static String frontCam = "limelight-front";
    
    public static Pigeon2 m_gyro;
    private static SwerveSubsystem SWERVE;

    /**
     * Inits static variables used by this class
     * 
     * @param Swerve our swerve subsystem
     */
    public LimeLightExtra(SwerveSubsystem Swerve) {
        this.SWERVE = Swerve;
        // estimator = SWERVE.getSwerveDrive().swerveDrivePoseEstimator;
        this.m_gyro = (Pigeon2) ((Pigeon2Swerve) SWERVE.getSwerveDrive().getGyro()).getIMU();
    }

    /**
     * Returns the best tag
     * 
     * 
     * @param limeLightName
     * @return
     */
    public static Optional<RawFiducial> getBestTag(String limeLightName) {
        RawFiducial[] tags = LimelightHelpers.getRawFiducials(limeLightName); // gets tags

        // if there are no tags, nothing happens
        if (tags.length == 0) {
            return Optional.empty();
        }

        // finds the clearest tag
        RawFiducial bestResult = tags[0];
        double amiguity = tags[0].ambiguity;
        // double currentAmbiguity = 0;

        for (RawFiducial tag : tags) {
            if (tag.ambiguity < amiguity) {
                bestResult = tag;
                amiguity = tag.ambiguity;
            }
        }

        System.out.println(bestResult.id);
        return Optional.of(bestResult);
    }

    /**
     * Updates the pose estimate of the best tag using MT2
     * 
     */
    public static void updatePoseEstimation() {
        boolean doRejectUpdate = false;
        LimelightHelpers.SetRobotOrientation("limelight", SWERVE.getHeading().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if(Math.abs(m_gyro.getRate()) > 180) // if our angular velocity is greater than 90 degrees per second, ignore vision updates
        {
          doRejectUpdate = true;
        } else if(mt2.tagCount == 0)
        {
          doRejectUpdate = true;
        } else if (mt2.pose.getX() == 0. && mt2.pose.getY() == 0.) {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            // SWERVE.getSwerveDrive().setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            SWERVE.getSwerveDrive().addVisionMeasurement(
              mt2.pose,
              mt2.timestampSeconds);
        }
    }

    // Requests a tag position
    public static Optional<double[]> requestTagPos(String limelight) {
        if (LimelightHelpers.getTV(limelight)) {
            //Limiter code here!
            return Optional.of(LimelightHelpers.getTargetPose_RobotSpace(limelight));
        } else {
            System.out.println("x");
            return Optional.empty();
        }
    }
    
    // prints variables from target space
    public static void printTargetSpace(String limeString) {
        for (double i : LimelightHelpers.getTargetPose_RobotSpace(limeString)) {
            System.out.println(i);
        };
        System.out.println("");
    }

    // returns if the requested tag is seen
    public static boolean seesTag(int tagID) {
        RawFiducial[] x = LimelightHelpers.getRawFiducials(backCam);

        for (RawFiducial tag : x) {
            if (tag.id == tagID) return true;
        } 

        return false;
    }
}