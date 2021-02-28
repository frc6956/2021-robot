package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Creates a new LinearSlide (The Robot Superstructure)
 */
public class LinearSlide extends SubsystemBase {
    private DoubleSolenoid m_slideSolenoid = new DoubleSolenoid(Constants.PCM.slideIn, Constants.PCM.slideOut);

    public LinearSlide() {

    }

    public void actuateOut() {
        if(m_slideSolenoid.get() != DoubleSolenoid.Value.kForward) {
            m_slideSolenoid.set(DoubleSolenoid.Value.kForward);
          }
      }
      public void actuateIn() {
        if (m_slideSolenoid.get() != DoubleSolenoid.Value.kReverse) {
            m_slideSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
      }
}