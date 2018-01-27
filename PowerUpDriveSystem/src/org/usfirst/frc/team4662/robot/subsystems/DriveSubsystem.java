package org.usfirst.frc.team4662.robot.subsystems;

import org.usfirst.frc.team4662.robot.Robot;
import org.usfirst.frc.team4662.robot.commands.ArcadeDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveSubsystem extends Subsystem {
	
	private WPI_TalonSRX m_leftController1;
	private WPI_TalonSRX m_leftController2;
	private WPI_TalonSRX m_rightController1;
	private WPI_TalonSRX m_rightController2;
	private SpeedControllerGroup m_leftControlGroup;
	private SpeedControllerGroup m_rightControlGroup;
	private DifferentialDrive m_robotDrive;
	private AHRS m_AHRSnavX;
	private PIDController m_turnAngle;
	private double m_dTurnAngleP;
	private double m_dTurnAngleI;
	private double m_dTurnAngleD;
	
	public DriveSubsystem() {
		
		m_leftController1 = new WPI_TalonSRX(Robot.m_robotMap.getPortNumber("leftController1"));
		m_leftController2 = new WPI_TalonSRX(Robot.m_robotMap.getPortNumber("leftController2"));
		m_rightController1 = new WPI_TalonSRX(Robot.m_robotMap.getPortNumber("rightController1"));
		m_rightController2 = new WPI_TalonSRX(Robot.m_robotMap.getPortNumber("rightController2"));
		m_leftControlGroup = new SpeedControllerGroup(m_leftController1, m_leftController2);
		m_rightControlGroup = new SpeedControllerGroup(m_rightController1, m_rightController2);
		m_leftControlGroup.setInverted(false);
		m_rightControlGroup.setInverted(false);
		m_robotDrive = new DifferentialDrive(m_leftControlGroup, m_rightControlGroup);
		
		m_AHRSnavX = new AHRS(SPI.Port.kMXP);
		m_dTurnAngleP = Robot.m_robotMap.getPIDPVal("TurnAngle", 0.2);
		m_dTurnAngleI = Robot.m_robotMap.getPIDIVal("TurnAngle", 0.4);
		m_dTurnAngleD = Robot.m_robotMap.getPIDDVal("TurnAngle", 0.4);
		m_turnAngle = new PIDController(m_dTurnAngleP, m_dTurnAngleI, m_dTurnAngleD, new getSourceAngle(), new putOutputTurn() );
	}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new ArcadeDrive());
    }
    
    public void arcadeDrive(double throttle, double turn) {
    	m_robotDrive.arcadeDrive(throttle * -1, turn);
    	smartDashBoardDiplay();
    }
    
    private void smartDashBoardDiplay() {
    	SmartDashboard.putNumber("navxGyro", m_AHRSnavX.getAngle() );
    }
    
    private double getGyroAngle() {
    	return m_AHRSnavX.getAngle();
    }
    
    private class getSourceAngle implements PIDSource {

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			// TODO Auto-generated method stub
			
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			// TODO Auto-generated method stub
			return PIDSourceType.kDisplacement;
		}

		@Override
		public double pidGet() {
			// TODO Auto-generated method stub
			return getGyroAngle();
		}
    	
    }
    
    private class putOutputTurn implements PIDOutput {

		@Override
		public void pidWrite(double output) {
			// TODO Auto-generated method stub
			arcadeDrive(0, output);
		}
    	
    }
}

