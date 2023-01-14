package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {
    PigeonIMU gyro = new PigeonIMU(20);
    SwerveModule[] modules = new SwerveModule[4]; 
    SwerveDriveKinematics m_kinematics;
    SwerveModuleState[]  moduleStates = new SwerveModuleState[4]; 
    double[] encPositionRad = new double[4];
    double maxVel=1; // mps

    // ****    The turn motor and encoder inversions weren't quite right 
    //  So the order of the modules and the gyro heading sign were fudged
    // order is normally FL-FR-BL-BR 

   SwerveDrive(){
    // name, drive motor ID, turn motor ID, mag encoder ID, ange zero pos
    modules[1]=new SwerveModule("FL", 2, 3, 3, 167.0,true);
    modules[0]=new SwerveModule("FR", 4, 5, 1, 110.0,false);
    modules[3]=new SwerveModule("BL", 8, 1, 2, 70.0,true);
    modules[2]=new SwerveModule("BR", 6, 7, 0, 82.0,true);
    m_kinematics = new SwerveDriveKinematics(
        new Translation2d(0.4064, 0.4064), 
        new Translation2d(0.4064, -0.4064), 
        new Translation2d(-0.4064, 0.4064), 
        new Translation2d(-0.4064, -0.4064)
        );

    resetGyro();

   }

// set motors   
   public void setModules(double joyMag,double joyAngle, double omega){
    double vx=joyMag*Math.sin(joyAngle*Math.PI/180);
    double vy= joyMag*Math.cos(joyAngle*Math.PI/180);
    double speedSet,turnSet;
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        vy, vx, omega,  new Rotation2d(-getHeadingDegrees()*Math.PI/180.));  
    moduleStates = m_kinematics.toSwerveModuleStates(speeds);    
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,maxVel);
    int i=0;
    while (i<=3){
        encPositionRad[i]=modules[i].getAbsAngleRad();
        speedSet=moduleStates[i].speedMetersPerSecond;
        turnSet=moduleStates[i].angle.getDegrees();
        modules[i].setMotors(speedSet, turnSet);
        modules[i].printState();
        i++;
    }
    SmartDashboard.putNumber("heading", getHeadingDegrees());
   }

   public void resetGyro() {
    gyro.setYaw(0);
   }

   public double getHeadingDegrees(){
    double[] ypr_deg = new double[3];
    gyro.getYawPitchRoll(ypr_deg);
    double angle=ypr_deg[0] % 360;
    if (angle>180) angle=angle-360;
    else if (angle <-180) angle = angle + 360;
    return angle;
} 


}
