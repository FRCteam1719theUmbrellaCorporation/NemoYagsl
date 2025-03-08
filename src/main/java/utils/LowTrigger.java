package utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LowTrigger extends Trigger {

  public LowTrigger(XboxController controller, XboxController.Axis axis) {
    super(() -> controller.getRawAxis(axis.value) <= -0.5);
  }

}