package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
 import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class SwerveModule {
    CANSparkMax driveMotor;
    CANSparkMax turnMotor;
    Counter absEncoder;
    RelativeEncoder driveEnc, turnEnc;
    SparkMaxPIDController driveController, turnController;
    double absEncConversion=87890.625;  // converts period to degrees
    double absEncConversionRad=87890.625*Math.PI/180.;  // converts period to radians
    String name;   
    double angleOffsetDeg, angleOffsetRev;     
    double driveSet=0,turnSet=0;
    int reverse=1;
    static double  
            kPturn=3.6/64,
            kDturn=0,
            kPdrive=0.00005,
            kFdrive=0.000175;

    SwerveModule(String _name, int driveID, int turnID, int EncID, double zeroAngle,
             boolean invDrive){
        name=_name;

        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setInverted(invDrive);
        driveEnc=driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        driveController.setFF(kFdrive);

        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        turnMotor.restoreFactoryDefaults();
        turnMotor.setIdleMode(IdleMode.kCoast);
        turnMotor.setInverted(false);
        turnEnc=turnMotor.getEncoder();
        turnEnc.setPosition(0);
        turnController = turnMotor.getPIDController();
        turnController.setP(kPturn);
        absEncoder = new Counter();
        absEncoder.setUpSource(new DigitalInput(EncID));
        absEncoder.setSemiPeriodMode(true);
        Timer.delay(.5);
        double initialAngle=getAbsAngle();
        angleOffsetDeg=zeroAngle-initialAngle;
        angleOffsetRev=angleOffsetDeg/360;
        SmartDashboard.putNumber(name+" zeroPos", zeroAngle);
        SmartDashboard.putNumber(name+" offset", angleOffsetDeg);

        SmartDashboard.putNumber("kP Turn", kPturn);
        SmartDashboard.putNumber("kD Turn", kDturn);
        SmartDashboard.putNumber("kF Drive", kFdrive);
        SmartDashboard.putNumber("kP Drive", kPdrive);

    }

// gets the turn module absolute angle in degrees    
    public double getAbsAngle(){
        return 360-absEncConversion*absEncoder.getPeriod();
    }

public double getAbsAngleRad(){
        return Math.PI-absEncConversionRad*absEncoder.getPeriod();
    }

// gets the turn module relative angle in rotations    
public double getRelRot(){
    return turnEnc.getPosition();
}


// gets the drive module distance in rotations    
public double getDist(){
    return driveEnc.getPosition();
}

// gets the drive module velocity in rpm    
public double getVel(){
    return driveEnc.getVelocity();
}

// set the drive and turn motors    
public void setMotors(double m_driveSet, double m_turnSet){
    driveSet=m_driveSet;turnSet=m_turnSet;
    driveController.setReference(reverse*driveSet*2000, ControlType.kVelocity);    
    double optRot=optimize();
//    System.out.println(optRot);
    turnController.setReference(optRot, ControlType.kPosition);    

    SmartDashboard.putNumber(name+" rev", reverse);

//  turnController.setReference(64*(turnSet/(360)+angleOffsetRev), ControlType.kPosition);
}

// print output    
public void printState(){    
 SmartDashboard.putNumber(name+" dist", getDist());
 SmartDashboard.putNumber(name+" vel", getVel());
 SmartDashboard.putNumber(name+" turnAbsAngle", getAbsAngle());
 SmartDashboard.putNumber(name+" turnEnc", getRelRot());
 SmartDashboard.putNumber(name+" relAngle",(360*getRelRot()/64.)%360);
 SmartDashboard.putNumber(name+" driveSet",driveSet);
 SmartDashboard.putNumber(name+" turnSet",turnSet);
 

 


}

public static void updateConstants(){
    kFdrive=SmartDashboard.getNumber("kF Drive", 0);
    kPdrive=SmartDashboard.getNumber("kP Drive", 0);
    kPturn=SmartDashboard.getNumber("kP Turn", 0);
    kDturn=SmartDashboard.getNumber("kD Turn", 0);
}

private double optimize(){
    reverse=1;
    double optimizedRotation;
    double initialRot=getRelRot();
    double initialAngle=360*((initialRot/64)%1);  // starting module angle from -180 to 180 
    if (initialAngle>180) {initialAngle=initialAngle-360;}
    double delta=(turnSet+angleOffsetDeg)-initialAngle;  // get angle difference in degrees
    if(delta >= 180) {delta = delta-360;}
    if(delta <= -360) {delta = delta + 360;}

    if(delta >= 90) {
        delta = delta-180;
        reverse=-1;
    }
    
    if(delta <= -90) {
        delta = delta + 180;
        reverse=-1;
    }

    optimizedRotation=initialRot+64*delta/360.;
    return optimizedRotation;
}


}
