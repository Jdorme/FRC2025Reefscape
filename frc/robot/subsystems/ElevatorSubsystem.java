// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final TalonFX m_motorR;
  private final TalonFX m_motorL;
  private final DutyCycleEncoder m_encoder1;
  
  double m_desiredSetpoint = Constants.ElevatorConstants.neutral;

  private ElevatorFeedforward m_feedforward;
  private PIDController m_pid;
  private final MotionMagicExpoTorqueCurrentFOC m_motionMagic;

  //Constructor
  public ElevatorSubsystem() {
    
    m_motorR = new TalonFX(Constants.ElevatorConstants.elevatorMotorRID, "rio"); //Canbus = "rio"?
    //REVERSEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEvvvvvvvvvvv
    m_motorL = new TalonFX(Constants.ElevatorConstants.elevatorMotorLID, "rio"); //<------REVERSE DIRECTION!!!!!!!!!!!!!!!!!!!!!
    //PLEASE REVERSE DIRECTION I BEG OF YOU PLEASE^^^^^^^^^^^
    
     m_encoder1 = new DutyCycleEncoder(Constants.ElevatorConstants.encoder1DIO); //Want to set range and expected zero?

    // m_feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
    // m_pid = new PIDController(kP, kI, kD);


  //Motion Magic---------------------------------------------------------------------
    m_motionMagic = new MotionMagicExpoTorqueCurrentFOC(0); //whats the positon for

  }

  public void setSetpoint(double setpoint){

    m_desiredSetpoint = setpoint;

  }

//Old .calculate plan
  // public void enable(double desiredSetpoint) {
    //   m_desiredSetpoint = desiredSetpoint; //Written like this so we can also grab this value outside of this method.
    //   m_motorR.set(m_feedforward.calculate(m_desiredSetpoint) + m_pid.calculate(m_encoder1.get(), m_desiredSetpoint)); //.set uses speed between 1 and 0
    //   m_motorL.set(m_feedforward.calculate(m_desiredSetpoint) + m_pid.calculate(m_encoder1.get(), m_desiredSetpoint));
    //   // m_motorR.setVoltage(m_pid.calculate(m_encoder1.get(), m_setpoint) + m_feedforward);
    //   // m_motorL.setVoltage(m_pid.calculate(m_encoder1.get(), m_setpoint) + m_feedforward);
    // }

  @Override
  public void periodic() {
  // This method will be called once per scheduler run

    // m_motorR.set(m_feedforward.calculate(m_desiredSetpoint) + m_pid.calculate(m_encoder1.get(), m_desiredSetpoint)); //.set uses speed between 1 and 0
    // m_motorL.set(m_feedforward.calculate(m_desiredSetpoint) + m_pid.calculate(m_encoder1.get(), m_desiredSetpoint));
    
  //Motion Magic-------------------------------------------------------------------------
    m_motorR.setControl(m_motionMagic.withPosition(m_desiredSetpoint).withFeedForward(null)); //what to do for feedforward


  }
}
