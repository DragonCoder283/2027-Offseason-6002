// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  private static final double kSpeedEpsilon = 0.01; // m/s, tune if needed
  // Margin around the 90-degree optimize() flip boundary. Once flipped, the error must swing
  // back within this margin of the *other* side before we'll flip again - prevents chatter
  // when the raw angle error sits right on the boundary due to sensor noise.
  private static final double kFlipHysteresisRad = Math.toRadians(5);

  private boolean lastFlipped = false;

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * wheelRadiusMeters;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
  }

  /** Runs the module with the specified setpoint state (no acceleration feedforward). */
  public void runSetpoint(SwerveModuleState state) {
    runSetpoint(state, 0.0);
  }

  /**
   * Runs the module with the specified setpoint state and acceleration feedforward (kA). Mutates
   * the state to optimize it, using hysteresis around the 90-degree flip boundary to avoid
   * chattering between the two equivalent solutions when the angle error sits near that line.
   */
  public void runSetpoint(SwerveModuleState state, double feedforwardAccelMPSSq) {
    if (Math.abs(state.speedMetersPerSecond) < kSpeedEpsilon) {
      io.setDriveVelocity(0.0, 0.0);
      return;
    }

    Rotation2d current = getAngle();
    double errRad = MathUtil.angleModulus(state.angle.minus(current).getRadians());

    // Decide whether to flip, with a deadband around the 90-degree boundary so noise near the
    // line doesn't cause the decision to chatter between calls.
    boolean flipped;
    if (lastFlipped) {
      // Currently flipped - only un-flip if we've moved clearly back to the non-flipped side
      flipped = Math.abs(errRad) > Math.PI / 2 - kFlipHysteresisRad;
    } else {
      // Currently not flipped - only flip if we've moved clearly past the boundary
      flipped = Math.abs(errRad) > Math.PI / 2 + kFlipHysteresisRad;
    }
    lastFlipped = flipped;

    if (flipped) {
      state.angle = state.angle.rotateBy(Rotation2d.kPi);
      state.speedMetersPerSecond *= -1;
    }
    state.cosineScale(current);

    double accelRadPerSecSq =
        (flipped ? -feedforwardAccelMPSSq : feedforwardAccelMPSSq) / wheelRadiusMeters;

    io.setDriveVelocity(state.speedMetersPerSecond / wheelRadiusMeters, accelRadPerSecSq);
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(Rotation2d.kZero);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * wheelRadiusMeters;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * wheelRadiusMeters;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rad/sec. */
  public double getFFCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }
}