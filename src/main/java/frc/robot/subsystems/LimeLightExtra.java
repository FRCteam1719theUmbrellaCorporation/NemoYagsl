package frc.robot.subsystems;


import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.imu.Pigeon2Swerve;
import swervelib.telemetry.SwerveDriveTelemetry;

public class LimeLightExtra {

    public final static String backCam = "limelight-back";
    public final static String frontCam = "limelight-front";
    
    private static Pigeon2 m_gyro;
    private static SwerveSubsystem SWERVE;
    private static SwerveDrivePoseEstimator estimator;


    public LimeLightExtra(SwerveSubsystem Swerve) {

        this.SWERVE = Swerve;
        estimator = SWERVE.getSwerveDrive().swerveDrivePoseEstimator;
        //this.m_gyro = gyro;
        this.m_gyro = (Pigeon2) ((Pigeon2Swerve) SWERVE.getSwerveDrive().getGyro()).getIMU();
    }

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

    

    public static void updatePoseEstimation() {
        boolean doRejectUpdate = false;
        LimelightHelpers.SetRobotOrientation("limelight", SWERVE.getHeading().getDegrees(), 0, 0, 0, 0, 0);
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