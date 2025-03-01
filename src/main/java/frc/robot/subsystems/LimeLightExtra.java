package frc.robot.subsystems;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

public class LimeLightExtra {

    public final static String backCam = null;
    //public final static String frontCam = "limelight-front";

    private static SwerveSubsystem SWERVE;
    private final boolean doRejectUpdate = true;



    // public LimeLightExtra(SwerveSubsystem Swerve) {
    //     this.SWERVE = Swerve;
    // }

    public static Optional<RawFiducial> getBestTag(String limeLightName) {
        RawFiducial[] tags = LimelightHelpers.getRawFiducials(limeLightName);
        if (tags.length == 0) {

            System.out.println("x");
            return Optional.empty();
        }

        RawFiducial bestResult = tags[0];
        double amiguity = tags[0].ambiguity;
        double currentAmbiguity = 0;

        for (RawFiducial tag : tags) {
            if (tag.ambiguity < amiguity) {
                bestResult = tag;
                amiguity = tag.ambiguity;
            }
        }

        System.out.println(bestResult.id);
        return Optional.of(bestResult);
    }

    // OLD? TODO: MEGA TAG 2
    public static void updatePoseEstimation(SwerveDrive swerveDrive) {


        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(backCam);
        boolean doRejectUpdate = false;

        if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
        {
            if(mt1.rawFiducials[0].ambiguity > .7)
            {
            doRejectUpdate = true;
            }
            if(mt1.rawFiducials[0].distToCamera > 3)
            {
            doRejectUpdate = true;
            }
        }
        if(mt1.tagCount == 0)
        {
            doRejectUpdate = true;
        }

        if(!doRejectUpdate)
        {
            swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
            swerveDrive.addVisionMeasurement(
                mt1.pose,
                mt1.timestampSeconds);
        }

    LimelightHelpers.SetRobotOrientation("limelight", swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    //if(Math.abs(swerveDrive.getGyro()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    //{
    //doRejectUpdate = true;
    //}
    if(mt2.tagCount == 0)
    {
    doRejectUpdate = true;
    }
    if(!doRejectUpdate)
    {
    swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
    swerveDrive.addVisionMeasurement(
        mt2.pose,
        mt2.timestampSeconds);
    }

    }

   /* public static updatePoseEstimation2(SwerveDrive swerveDrive){
        LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
*/
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