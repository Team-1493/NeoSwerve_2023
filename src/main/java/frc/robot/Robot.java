
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  SwerveDrive sd = new SwerveDrive();
  Joystick joy0=new Joystick(0);

  @Override
  public void robotInit() {

  }

 

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }


  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
        // update constants
        if (joy0.getRawButton(1)) sd.resetGyro();

        //  read the joystick magnitude and angle in degrees
        double joyMag=joy0.getMagnitude();
        joyMag=joyMag*joyMag;
        double joyAngle = joy0.getDirectionDegrees();
        double omega=joy0.getRawAxis(2)*0.5;
        SmartDashboard.putNumber("Joystick Angle", joyAngle);
        SmartDashboard.putNumber("Joystick Angle", joy0.getRawAxis(0));
        SmartDashboard.putNumber("Joystick Mag", joyMag);
        
        // set the swerve module motors
        sd.setModules(joyMag, joyAngle,omega);
    
  }

}
