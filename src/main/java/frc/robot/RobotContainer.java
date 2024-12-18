// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;// HID stands for Human interface device.
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.BlinkLED;
import frc.robot.commands.ChaseLED;
import frc.robot.commands.DefaultTeleop;
import frc.robot.commands.GoStraight;
import frc.robot.commands.GoToApriltag;
import frc.robot.commands.RumbleController;
import frc.robot.commands.SetForwardToTowardsFront;
import frc.robot.commands.Spin;
import frc.robot.commands.XmasPattern;
import frc.robot.subsystems.ControllerRumbler;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.ApriltagInfo;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final OmniWheel m_ow = new OmniWheel(new TalonSRX(3), "default"); // the 2 was 1 -> this change does change motor controller lights
  // private final RunWheel m_rw = new RunWheel(m_ow, -0.5, 5.0); // the -0.5 was 1.0
  private final XboxController m_controller = new XboxController(0);
  private final ControllerRumbler m_rumbler = new ControllerRumbler(m_controller);
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final LEDStrip m_ledStrip = new LEDStrip(0, 60);
  private final ApriltagInfo m_apriltagInfo = new ApriltagInfo(4173, "rPi", new int[]{1, 2, 3, 4, 5, 6, 7, 8});
    
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveTrain.setName("The Drive Train");
    SmartDashboard.putData(m_driveTrain); // show commands controlling the drive train
    
    m_ledStrip.setName("the led strip");
    SmartDashboard.putData(m_ledStrip);
    
    // m_SparkMaxBrushless.setPIDCoefficients(0.000080, 0.0, 0.0, 0.0, 0.000170, -1.0, 1.0);
    configureButtonBindings();
    m_driveTrain.setDefaultCommand(new DefaultTeleop(m_driveTrain, m_controller, m_rumbler));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Y button makes robot go forward at 1.5 feet/second, until button is released
    // 2022->2023 wpilib changes in JoystickButton class:
    //   * whenPressed renamed to onTrue
    //   * get renamed to getAsBoolean
    JoystickButton yButton = new JoystickButton(m_controller, XboxController.Button.kY.value);
    yButton.whileTrue(new GoStraight(m_driveTrain, 1.5, 0.0));
    // A button makes robot go left at 1.3 feet/second, until button is released
    JoystickButton aButton = new JoystickButton(m_controller, XboxController.Button.kA.value);
    aButton.whileTrue(new GoStraight(m_driveTrain, 1.3, -90.));
    JoystickButton backButton = new JoystickButton(m_controller, XboxController.Button.kBack.value);
    // Pressing 'back' button once causes left rumble for 1 sec., then right rumble 1 sec., then both gently for 1 sec.
    backButton.onTrue(
                      new RumbleController(m_rumbler, 1.0, 0.0).withTimeout(1.0)
            .andThen( new RumbleController(m_rumbler, 0.0, 1.0).withTimeout(1.0))
            .andThen( new RumbleController(m_rumbler, 0.5, 0.5).withTimeout(1.0))
            );
    JoystickButton startButton = new JoystickButton(m_controller, XboxController.Button.kStart.value);
    startButton.onTrue(new SetForwardToTowardsFront(m_driveTrain));

    // Pressing 'B' causes it to go to AprilTag 1.  Interrupt search when button is released
    JoystickButton bButton = new JoystickButton(m_controller, XboxController.Button.kB.value);
    bButton.whileTrue(
      new GoToApriltag(1, m_driveTrain, m_apriltagInfo)
      .andThen(new RumbleController(m_rumbler, 0.5, 0.5))
      );
    // 'X' toggles blinking of LEDs
    JoystickButton xButton = new JoystickButton(m_controller, XboxController.Button.kX.value);
    xButton.toggleOnTrue(new BlinkLED(m_ledStrip,
      new double[]{0.5, 0.25},
      new Color[][]{
        new Color[]{ Color.kBlue, Color.kBlue, Color.kBlue, Color.kYellow},
        new Color[]{ Color.kRed, Color.kGreen, Color.kGreen}}));
    new Trigger(() -> m_controller.getPOV() == 180).toggleOnTrue(new ChaseLED(m_ledStrip,
      0.1,
      ChaseLED.Direction.kOut, 
      new Color[]{ Color.kViolet, Color.kBlueViolet, Color.kBlue, Color.kAqua,
        Color.kGreen, Color.kYellowGreen, Color.kYellow, Color.kWhite})
      );
    // right 'bumper' toggles Xmas lights
    // (We've run out of buttons on the Xbox controller - the 
    // drive train's default command uses the buttons to do
    // slow rotations.)
    JoystickButton xmasTriggerOut = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    xmasTriggerOut.whileTrue(new XmasPattern(m_ledStrip, 0.125, LEDStrip.MotionType.kRotateOut)); 
    JoystickButton xmasTriggerIn = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
    xmasTriggerIn.whileTrue(new XmasPattern(m_ledStrip, 0.125, LEDStrip.MotionType.kRotateIn )); 
  } 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // very slowly spin for 5 seconds
    return new Spin(m_driveTrain, 0.4).withTimeout(5.0);
  }
}
