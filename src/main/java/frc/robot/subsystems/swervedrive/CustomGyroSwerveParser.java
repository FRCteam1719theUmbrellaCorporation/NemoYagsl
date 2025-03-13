package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.parser.json.ModuleJson;
import utils.AwfulGyroWrapper;

public class CustomGyroSwerveParser extends SwerveParser {
    public CustomGyroSwerveParser(File directory) throws IOException {
        super(directory);
    }

    public SwerveDrive createSwerveDrive(double maxSpeed, Pose2d initialPose) {
    SwerveModuleConfiguration[] moduleConfigurations =
        new SwerveModuleConfiguration[moduleJsons.length];
    for (int i = 0; i < moduleConfigurations.length; i++) {
      ModuleJson module = moduleJsons[i];
      moduleConfigurations[i] =
          module.createModuleConfiguration(
              pidfPropertiesJson.angle,
              pidfPropertiesJson.drive,
              physicalPropertiesJson.createPhysicalProperties(),
              swerveDriveJson.modules[i]);
    }
    SwerveDriveConfiguration swerveDriveConfiguration =
        new SwerveDriveConfiguration(
            moduleConfigurations,
            new AwfulGyroWrapper(swerveDriveJson.imu.id, swerveDriveJson.imu.canbus == null ? swerveDriveJson.imu.canbus : ""),
            swerveDriveJson.invertedIMU,
            physicalPropertiesJson.createPhysicalProperties());

    return new SwerveDrive(
        swerveDriveConfiguration,
        controllerPropertiesJson.createControllerConfiguration(swerveDriveConfiguration, maxSpeed),
        maxSpeed,
        initialPose);
  }
}