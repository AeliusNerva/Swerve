/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.OI;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

/**
 * Add your docs here.
 */

public class DriveSubsystem extends Subsystem {
	AHRS ahrs;
	Joystick _joy = new Joystick(0);
	double radians, angle, temp, A2, B2, R, A, B, C, D, ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4, max, currentAngle,
			rotationAmmount, FWD, STR, RCW, currentAngle2, currentAngle3, currentAngle4, rotationAmmount2,
			rotationAmmount3, rotationAmmount4; // defining variables formulas

	// defining motor controlers and encoders
	WPI_TalonSRX SRXsteer = new WPI_TalonSRX(OI.SRXtoprightsteer);
	WPI_TalonSRX SRXsteer2 = new WPI_TalonSRX(OI.SRXtopleftsteer);
	WPI_TalonSRX SRXsteer3 = new WPI_TalonSRX(OI.SRXbottomleftsteer);
	WPI_TalonSRX SRXsteer4 = new WPI_TalonSRX(OI.SRXbottomrightsteer);
	CANSparkMax MAXdrive = new CANSparkMax(OI.MAXtoprightdrive, MotorType.kBrushless);
	CANSparkMax MAXdrive2 = new CANSparkMax(OI.MAXtopleftdrive, MotorType.kBrushless);
	CANSparkMax MAXdrive3 = new CANSparkMax(OI.MAXbottomleftdrive, MotorType.kBrushless);
	CANSparkMax MAXdrive4 = new CANSparkMax(OI.MAXbottomrightdrive, MotorType.kBrushless);
	public CANPIDController drive1 = new CANPIDController(MAXdrive);
	public CANEncoder Encoder = new CANEncoder(MAXdrive);
	// defining motor controlers and encoders

	@Override
	public void initDefaultCommand() {
		MAXdrive.setMotorType(MotorType.kBrushless); // defining motor type
	}

	public void init() {
		// init for navX imu
		ahrs = new AHRS(SPI.Port.kMXP);
		ahrs.resetDisplacement();
		ahrs.reset();
		// init for navX imu

		// PID config
		drive1.setP(1);
		drive1.setI(0);
		drive1.setD(0);

		int kTimeoutMs = 10;
		int kPIDLoopIdx = 0;

		int absolutePosition = SRXsteer.getSelectedSensorPosition(kTimeoutMs) & 0xFFF;

		SRXsteer.setSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);
		SRXsteer.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
		SRXsteer.setSensorPhase(false);
		SRXsteer.selectProfileSlot(0, kPIDLoopIdx);
		SRXsteer.configNominalOutputForward(0, kTimeoutMs);
		SRXsteer.configNominalOutputReverse(0, kTimeoutMs);
		SRXsteer.configPeakOutputForward(1, kTimeoutMs);
		SRXsteer.configPeakOutputReverse(-1, kTimeoutMs);
		SRXsteer.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);
		SRXsteer.configMotionCruiseVelocity(15000, kTimeoutMs);
		SRXsteer.configMotionAcceleration(16000, kTimeoutMs);

		SRXsteer.config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		SRXsteer.config_kP(kPIDLoopIdx, 1, kTimeoutMs);
		SRXsteer.config_kI(kPIDLoopIdx, 0, kTimeoutMs);
		SRXsteer.config_kD(kPIDLoopIdx, 0, kTimeoutMs);
		SRXsteer.setSelectedSensorPosition(0, 0, 0);

		int absolutePosition2 = SRXsteer2.getSelectedSensorPosition(kTimeoutMs) & 0xFFF;

		SRXsteer2.setSelectedSensorPosition(absolutePosition2, kPIDLoopIdx, kTimeoutMs);
		SRXsteer2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
		SRXsteer2.setSensorPhase(false);

		SRXsteer2.configNominalOutputForward(0, kTimeoutMs);
		SRXsteer2.configNominalOutputReverse(0, kTimeoutMs);
		SRXsteer2.configPeakOutputForward(1, kTimeoutMs);
		SRXsteer2.configPeakOutputReverse(-1, kTimeoutMs);
		SRXsteer2.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);
		SRXsteer2.configMotionCruiseVelocity(15000, kTimeoutMs);
		SRXsteer2.configMotionAcceleration(16000, kTimeoutMs);

		SRXsteer2.config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		SRXsteer2.config_kP(kPIDLoopIdx, 1, kTimeoutMs);
		SRXsteer2.config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		SRXsteer2.config_kD(kPIDLoopIdx, 0, kTimeoutMs);
		SRXsteer2.setSelectedSensorPosition(0, 0, 0);

		int absolutePosition3 = SRXsteer3.getSelectedSensorPosition(kTimeoutMs) & 0xFFF;

		SRXsteer3.setSelectedSensorPosition(absolutePosition3, kPIDLoopIdx, kTimeoutMs);
		SRXsteer3.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
		SRXsteer3.setSensorPhase(false);

		SRXsteer3.configNominalOutputForward(0, kTimeoutMs);
		SRXsteer3.configNominalOutputReverse(0, kTimeoutMs);
		SRXsteer3.configPeakOutputForward(1, kTimeoutMs);
		SRXsteer3.configPeakOutputReverse(-1, kTimeoutMs);
		SRXsteer3.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);
		SRXsteer3.configMotionCruiseVelocity(15000, kTimeoutMs);
		SRXsteer3.configMotionAcceleration(16000, kTimeoutMs);

		SRXsteer3.config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		SRXsteer3.config_kP(kPIDLoopIdx, 1, kTimeoutMs);
		SRXsteer3.config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		SRXsteer3.config_kD(kPIDLoopIdx, 0, kTimeoutMs);
		SRXsteer3.setSelectedSensorPosition(0, 0, 0);

		int absolutePosition4 = SRXsteer4.getSelectedSensorPosition(kTimeoutMs) & 0xFFF;

		SRXsteer4.setSelectedSensorPosition(absolutePosition4, kPIDLoopIdx, kTimeoutMs);
		SRXsteer4.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
		SRXsteer4.setSensorPhase(false);

		SRXsteer4.configNominalOutputForward(0, kTimeoutMs);
		SRXsteer4.configNominalOutputReverse(0, kTimeoutMs);
		SRXsteer4.configPeakOutputForward(1, kTimeoutMs);
		SRXsteer4.configPeakOutputReverse(-1, kTimeoutMs);
		SRXsteer4.configMotionCruiseVelocity(15000, kTimeoutMs);
		SRXsteer4.configMotionAcceleration(16000, kTimeoutMs);

		SRXsteer4.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

		SRXsteer4.config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		SRXsteer4.config_kP(kPIDLoopIdx, 1, kTimeoutMs);
		SRXsteer4.config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		SRXsteer4.config_kD(kPIDLoopIdx, 0, kTimeoutMs);
		SRXsteer4.setSelectedSensorPosition(0, 0, 0);

		// PID config

	}

	public void Swerve() {
		angle = ahrs.getYaw(); // defining variable for gyro
		radians = angle * Math.PI / 180;
		currentAngle = SRXsteer.getSelectedSensorPosition(0) / 25.930555555555; // setting the current angle of the
																				// wheel 11.4666 = tick per rotation/360
		currentAngle2 = SRXsteer2.getSelectedSensorPosition(0) / 25.930555555555;
		currentAngle3 = SRXsteer3.getSelectedSensorPosition(0) / 25.930555555555;
		currentAngle4 = SRXsteer4.getSelectedSensorPosition(0) / 25.930555555555;
		// swerve formulas
		FWD = -_joy.getRawAxis(1);
		STR = _joy.getRawAxis(0);
		RCW = _joy.getRawAxis(2);
		if (FWD < .08 && FWD > -.08) {
			FWD = 0;
		}
		if (STR < .08 && STR > -.08) {
			STR = 0;
		}
		if (RCW < .08 && RCW > -.08) {
			RCW = 0;
		}
		temp = FWD * Math.cos(radians) + STR * Math.sin(radians);
		STR = -FWD * Math.sin(radians) + STR * Math.cos(radians);
		FWD = temp;
		R = Math.sqrt((OI.L * OI.L) + (OI.W * OI.W));
		A = STR - RCW * (OI.L / R);
		B = STR + RCW * (OI.L / R);
		C = FWD + RCW * (OI.W / R);
		D = FWD + RCW * (OI.W / R);
		A2 = STR + RCW * (OI.L / R);
		B2 = STR - RCW * (OI.L / R);
		ws1 = Math.sqrt((B2 * B2) + (C * C));
		ws2 = Math.sqrt((B * B) + (D * D));
		ws3 = Math.sqrt((A * A) + (D * D));
		ws4 = Math.sqrt((A2 * A2) + (C * C));
		wa1 = Math.atan2(B2, C) * 180 / Math.PI;
		wa2 = Math.atan2(B, D) * 180 / Math.PI;
		wa3 = Math.atan2(A, D) * 180 / Math.PI;	
		wa4 = Math.atan2(A2, C) * 180 / Math.PI;
		max = ws1;
		if (ws2 > max) {
			max = ws2;
		}
		if (ws3 > max) {
			max = ws3;
		}
		if (ws4 > max) {
			max = ws4;
		}
		if (max > 1) {
			ws1 /= max;
			ws2 /= max;
			ws3 /= max;
			ws4 /= max;
		}
		if (RCW != 0) {
			ws1 = -ws1;
			ws4 = -ws4;
		}

		// swerve formulas

		// smartdashboard puts

		// smartdashboard puts
		SmartDashboard.putNumber("Encoder4 Position", SRXsteer4.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Encoder3 Position", SRXsteer3.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Encoder Position", SRXsteer.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Encoder2 Position", SRXsteer2.getSelectedSensorPosition(0));

		SmartDashboard.putNumber("angle4", currentAngle4);
		SmartDashboard.putNumber("output4", rotationAmmount4);
		SmartDashboard.putNumber("trtwt4", wa4 - currentAngle4);
		SmartDashboard.putNumber("angle3", currentAngle);
		SmartDashboard.putNumber("output3", rotationAmmount3);
		SmartDashboard.putNumber("trtwt3", wa4 - currentAngle3);

		SmartDashboard.putNumber("WheelAngle", wa1);
		SmartDashboard.putNumber("WheenSpeed", ws1);
		SmartDashboard.putNumber("WheelAngle2", wa2);
		SmartDashboard.putNumber("WheenSpeed2", ws2);
		SmartDashboard.putNumber("WheelAngle3", wa3);
		SmartDashboard.putNumber("WheenSpeed3", ws3);
		SmartDashboard.putNumber("WheelAngle4", wa4);
		SmartDashboard.putNumber("WheenSpeed4", ws4);
		SmartDashboard.putNumber("RotaitonAmount", rotationAmmount);
		SmartDashboard.putNumber("current", SRXsteer4.getOutputCurrent());
		SmartDashboard.putNumber("angle", angle);
		SmartDashboard.putNumber("joyasi1", _joy.getRawAxis(1));
		SmartDashboard.putNumber("joyasi0", _joy.getRawAxis(0));
		SmartDashboard.putNumber("joyasi2", _joy.getRawAxis(2));
		// System.out.println(rotationAmmount);

		rotationAmmount = Math.IEEEremainder(wa1 - currentAngle, 360); // calculating ammount to move wheel
		rotationAmmount2 = Math.IEEEremainder(wa2 - currentAngle2, 360); // calculating ammount to move wheel
		rotationAmmount3 = Math.IEEEremainder(wa3 - currentAngle3, 360); // calculating ammount to move wheel
		rotationAmmount4 = Math.IEEEremainder(wa4 - currentAngle4, 360); // calculating ammount to move wheel

		MAXdrive.setRampRate(.25);
		MAXdrive2.setRampRate(.25);
		MAXdrive3.setRampRate(.25);
		MAXdrive4.setRampRate(.25);

		if (FWD < .1 && FWD > -.1 && STR < .1 && STR > -.1 && RCW < .1 && RCW > -.1) { // not letting the wheels move
																						// unitll the joystick is pushed
																						// %10 down
			MAXdrive.set(0);
			MAXdrive2.set(0);
			MAXdrive3.set(0);
			MAXdrive4.set(0);
		} else {
	//		MAXdrive.set(-ws1); // drining drive wheel off the formulas
	//		MAXdrive2.set(ws2);
	//		MAXdrive3.set(ws3);
	//		MAXdrive4.set(-ws4);
		}

	//		SRXsteer.set(ControlMode.MotionMagic, (currentAngle + rotationAmmount) * 26.006); // moving the wheel to the
			// required position
	//		SRXsteer2.set(ControlMode.MotionMagic, (currentAngle2 + rotationAmmount2) * 26.006); // moving the wheel to
																									// the
			// required position
	//		SRXsteer3.set(ControlMode.MotionMagic, (currentAngle3 + rotationAmmount3) * 26.006); // moving the wheel to
																									// the
			// required position
	//		SRXsteer4.set(ControlMode.MotionMagic, (currentAngle4 + rotationAmmount4) * 26.006); // moving the wheel to
																									// the
			// required position

		/*
		 * SRXsteer.configPeakCurrentLimit(25, 10);
		 * SRXsteer.configPeakCurrentDuration(200, 10);
		 * SRXsteer.configContinuousCurrentLimit(10, 10);
		 * SRXsteer.enableCurrentLimit(true); SRXsteer2.configPeakCurrentLimit(25, 10);
		 * SRXsteer2.configPeakCurrentDuration(200, 10);
		 * SRXsteer2.configContinuousCurrentLimit(10, 10);
		 * SRXsteer2.enableCurrentLimit(true); SRXsteer3.configPeakCurrentLimit(25, 10);
		 * SRXsteer3.configPeakCurrentDuration(200, 10);
		 * SRXsteer3.configContinuousCurrentLimit(10, 10);
		 * SRXsteer3.enableCurrentLimit(true); SRXsteer4.configPeakCurrentLimit(25, 10);
		 * SRXsteer4.configPeakCurrentDuration(200, 10);
		 * SRXsteer4.configContinuousCurrentLimit(10, 10);
		 * SRXsteer4.enableCurrentLimit(true);
		 */

		SmartDashboard.putNumber("ANGLE", ahrs.getAngle());
	}

	public void driveForward(double distance) {
		currentAngle = SRXsteer.getSelectedSensorPosition(0) / 26.006; // setting the current angle of the wheel 11.4666
																		// = tick per rotation/360
		rotationAmmount = Math.IEEEremainder(0 - currentAngle, 360); // calculating ammount to move wheel
		SRXsteer.set(ControlMode.Position, (currentAngle + rotationAmmount) * 26.006); // moving the wheel to the
																						// required position
		drive1.setReference(distance, ControlType.kPosition); // moving drive to target position
	}
}
