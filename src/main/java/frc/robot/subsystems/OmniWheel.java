// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
//import frc.robot.TuningVariables;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
public class OmniWheel extends SubsystemBase {
  private final TalonSRX m_talonsrx;
  private double m_fractionPower;
  private final String m_name;
  private final double m_diameter;
   // The triomnidrive robot has US Digital E4P-360-250 encoders.
   //    360 cycles (== 1440 clicks) per revolution and hole for quarter inch shaft.
  private final double m_clicksPerRevolution = 360 * 4;
  private final double m_nativeUnitsOverRevolutionsPerSecond;
  private final double m_nativeUnitsOverFeetPerSecond;  
  private double m_kF;
  /** Creates a new OmniWheel, controlled by a TalonSRX motor controller
   *  @param canID:  CAN bus identifier for the motor controller
   *  @param name: a name for the wheel to be used in SmartDashboard displays
   *  @param diameter: diameter of wheel in feet */ 
    // eventually will need to specify diameter of wheel.
  public OmniWheel(int canId, String name, double diameter) {
    m_talonsrx = new TalonSRX(canId);
    m_fractionPower = 0;
    m_name = name;
    m_diameter = diameter;
    m_talonsrx.configFactoryDefault();
    m_talonsrx.clearStickyFaults();
    m_nativeUnitsOverRevolutionsPerSecond = m_clicksPerRevolution * 
                                            0.1 ;    // seconds per 100ms
    m_nativeUnitsOverFeetPerSecond = 1.0 / (Math.PI * m_diameter) *    // revolutions per feet travelled
                                     m_nativeUnitsOverRevolutionsPerSecond ;
    m_talonsrx.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.TalonSRX.kDriveTrainPIDLoopIndex, Constants.TalonSRX.kTimeoutMs);
    m_talonsrx.setSensorPhase(true);
    // for robot up on blocks, kF=1.23 and kP1.6 give ok results.
    // The question is, why?  Won't kF change when we put robot on floor?
    // Later I found that kI is essential when kF is not known well.
    // When on blocks typical accumulated deficits were 25-75 thousand when
    // requested speed was 150 or 200.
    // The above was written by me a few years ago.  It is wrong.  I should
    // figure out a better (bigger, I think - units are Volts/SpeedUnit) kF and set kI to 0.
    setKF(1.15);
    m_talonsrx.config_kP(Constants.TalonSRX.kDriveTrainPIDLoopIndex, 1.2);
    m_talonsrx.config_kI(Constants.TalonSRX.kDriveTrainPIDLoopIndex, 0.0);
    m_talonsrx.config_kD(Constants.TalonSRX.kDriveTrainPIDLoopIndex, 0.0); 
    SmartDashboard.putData(this); 
  }

  /**
   * set feed forward parameter.
   * @param kF feed forward parameter.  Units are Volts per Clicks/100Milliseconds.
   */
  private void setKF(double kF){
    m_kF = kF;
    m_talonsrx.config_kF(Constants.TalonSRX.kDriveTrainPIDLoopIndex, m_kF); 
  }
 
  /** Set the speed of the wheel on a scale of-1 to 1. 
      @param fractionPower: -1 is maximum power in reverse, 1 is maximum power forward.
        0 means stopped.
      */
  public void setFractionPower(double fractionPower) {
    m_fractionPower = fractionPower;
    m_talonsrx.set(ControlMode.PercentOutput,m_fractionPower);
  }

  public void setVelocityRevolutionsPerSecond(double revolutionsPerSecond){
    double clicksPer100Ms = revolutionsPerSecond * m_nativeUnitsOverRevolutionsPerSecond;
    setVelocityNativeUnits(clicksPer100Ms);
  }
  public void setVelocityFeetPerSecond(double feetPerSecond){
    double clicksPer100Ms = feetPerSecond * m_nativeUnitsOverFeetPerSecond;
    setVelocityNativeUnits(clicksPer100Ms);
  }

  /** set target velocity in "native" units: ticks per 100 milliseconds */
  public void setVelocityNativeUnits(double clicksPer100Ms) {
    m_talonsrx.set(ControlMode.Velocity, clicksPer100Ms);
  }

  @Override
  public void periodic() {
    // perhaps use this to collect sensor information to avoid double collection.
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder); // is this needed? or desirable?
    builder.addDoubleProperty("percentPower", () -> m_talonsrx.getMotorOutputPercent(), null);
    builder.addDoubleProperty("velocity", () -> m_talonsrx.getSelectedSensorVelocity(), null);
    builder.addDoubleProperty("integral accumulation",() -> m_talonsrx.getIntegralAccumulator(), null);
    builder.addDoubleProperty("kF", () -> m_kF, this::setKF);
    // builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);     builder.addDoubleProperty("velocity", m_talonsrx.getSelectedSensorVelocity());
  }
}
