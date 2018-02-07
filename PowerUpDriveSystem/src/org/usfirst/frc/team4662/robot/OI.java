/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4662.robot;

import org.usfirst.frc.team4662.robot.commands.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	public Joystick m_driveStick;
	public JoystickButton m_keepHeading;
	
	public OI() {
		
		if( Robot.m_robotMap.isDashboardTest()) {
			SmartDashboard.putData("PIDTest", new TurnAnglePID());
			SmartDashboard.putData("DriveDistancePID", new DriveDistancePID());
		}
		
		m_driveStick = new Joystick(0);
		m_keepHeading = new JoystickButton(m_driveStick,2);
		m_keepHeading.whileHeld(new KeepHeadingPID());
	}
}
