// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.falconfury.frc2025.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SwerveModule {
  // Swerve Module Configuration
  private final Configuration configuration_;

  // Motor Controllers
  private final SparkMax drive_motor_;
  private final SparkMax steer_motor_;

  // Configs
  private final SparkMaxConfig drive_motor_config_;
  private final SparkMaxConfig steer_motor_config_;
  private final CANcoderConfiguration cancoder_config_;

  // Sensors
  private final RelativeEncoder drive_encoder_;
  private final RelativeEncoder steer_encoder_;
  private final CANcoder cancoder_;

  // Control
  private final PIDController drive_pid_controller_; // in m/s
  private final PIDController steer_pid_controller_; // in radians
  
  /** Creates a new SwerveModule. */
  public SwerveModule(Configuration configuration) {
    // Store configuration
    configuration_ = configuration;

    // Initialize Motor Controllers
    drive_motor_ = new SparkMax(configuration_.drive_id, MotorType.kBrushless);
    drive_motor_config_ = new SparkMaxConfig();
    steer_motor_ = new SparkMax(configuration_.steer_id, MotorType.kBrushless);
    steer_motor_config_ = new SparkMaxConfig();

    // initialize Encoders
    drive_encoder_ = drive_motor_.getEncoder();
    steer_encoder_ = steer_motor_.getEncoder();

    // Configure drive motors
    drive_motor_config_
      .inverted(configuration_.invert_drive)
      .idleMode(IdleMode.kBrake);
    drive_motor_config_.encoder
      .positionConversionFactor(2 * Math.PI * Constants.kWheelRadius / Constants.kDriveGearRatio) // in meters
      .velocityConversionFactor(2 * Math.PI * Constants.kWheelRadius / Constants.kDriveGearRatio / 60.0); // in m/s

    // Configure steer motors
    steer_motor_config_
      .idleMode(IdleMode.kCoast)
      .inverted(true);
    steer_motor_config_.encoder
      .positionConversionFactor(2 * Math.PI / Constants.kSteerGearRatio)
      .velocityConversionFactor(2 * Math.PI / Constants.kSteerGearRatio / 60.0);

    // Initialize CANcoder
    cancoder_ = new CANcoder(configuration.cancoder_id);
    cancoder_config_ = new CANcoderConfiguration();
    cancoder_.getConfigurator().apply(cancoder_config_);

    // Initialize PID Controllers
    drive_pid_controller_ = new PIDController(Constants.kDriveKp, 0.0, 0.0);
    steer_pid_controller_ = new PIDController(Constants.kSteepKp, 0.0, 0.0);
    steer_pid_controller_.enableContinuousInput(-Math.PI, Math.PI);

    // Reset encoders
    resetEncoders();
  }

  // Get Drive Position
  public double getDrivePosition() {
    return drive_encoder_.getPosition();
  }

  // Get Steer Position
  public Rotation2d getSteerPosition() {
    return new Rotation2d(
      Math.IEEEremainder(steer_encoder_.getPosition(), 2 * Math.PI));
  }

  // Get Module Position
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getDrivePosition(), getSteerPosition());
  }

  // Get Drive Velocity
  public double getDriveVelocity() {
    return drive_encoder_.getVelocity();
  }

  // Get CANCoder Absolute Position in Degrees
  public double getCANCoderDeg() {
    return cancoder_.getAbsolutePosition().getValueAsDouble() - configuration_.module_offset_deg;
  }

  // Get Module State
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getDriveVelocity(), getSteerPosition());
  }

  // Reset Encoders
  public void resetEncoders() {
    drive_encoder_.setPosition(0.0);
    steer_encoder_.setPosition(Math.toRadians(getCANCoderDeg()));
  }

  /**
   * Sets Steer angle for individual module
   * @param desired_angle In Degrees
   */
  public void setAngle(double desired_angle_degs) {
    // Convert deg to rad
    double desired_angle_rad = Math.toRadians(desired_angle_degs);

    // Set steer angle
    double steering_correction = steer_pid_controller_.calculate(
      getSteerPosition().getRadians(), desired_angle_rad);
    steer_motor_.set(steering_correction);
  }

  // Set Desired State
  public void setDesiredState(SwerveModuleState state, Drive.OutputType output_type) {
    // Optimize state to minimize change in angle
    state.optimize(getModuleState().angle);

    // Set steer angle
    double steering_correction = steer_pid_controller_.calculate(
      getSteerPosition().getRadians(), state.angle.getRadians());
    steer_motor_.set(steering_correction);

    // Set drive output
    switch (output_type) {
      case OPEN_LOOP:
        drive_motor_.set(state.speedMetersPerSecond / Constants.kMaxModuleSpeed);
        break;
      case VELOCITY:
        double drive_correction = drive_pid_controller_.calculate(
          getDriveVelocity(), state.speedMetersPerSecond);
        drive_motor_.set(drive_correction);
        break;
    }
  }

  // Module Configuration
  public static class Configuration {
    public int drive_id;
    public int steer_id;
    public int cancoder_id;

    public double module_offset_deg;
    public boolean invert_drive;
  }

  // Constants Class
  public static class Constants {
    // Control
    public static final double kDriveKp = 0.5;
    public static final double kSteepKp = 0.5;
    
    // Hardware
    public static final double kDriveGearRatio = 0.0;
    public static final double kSteerGearRatio = 150.0 / 7;
    public static final double kWheelRadius = 0.0508;
    public static final double kMaxModuleSpeed = 0.0;
  }
  
}
