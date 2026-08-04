// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.LoggedTunableNumber;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  // Tunable AutoBuilder path-following PID gains.
  // NOTE: AutoBuilder.configure() can only be called ONCE - calling it again
  // logs "AutoBuilder has already been configured" and silently does nothing.
  // PPHolonomicDriveController also has no live setPID method. So instead of
  // rebuilding the controller and re-calling AutoBuilder.configure(), we hand
  // AutoBuilder a TunablePPController wrapper ONE time; the wrapper's inner
  // PPHolonomicDriveController can be swapped out freely whenever the tunables
  // change, without AutoBuilder ever needing to be reconfigured.
  private static final LoggedTunableNumber autoDriveKp =
      new LoggedTunableNumber("Drive/Auto/DriveKp", DriveConstants.driveKpAuto);
  private static final LoggedTunableNumber autoDriveKi =
      new LoggedTunableNumber("Drive/Auto/DriveKi", DriveConstants.driveKiAuto);
  private static final LoggedTunableNumber autoDriveKd =
      new LoggedTunableNumber("Drive/Auto/DriveKd", DriveConstants.driveKdAuto);
  private static final LoggedTunableNumber autoTurnKp =
      new LoggedTunableNumber("Drive/Auto/TurnKp", DriveConstants.turnKpAuto);
  private static final LoggedTunableNumber autoTurnKi =
      new LoggedTunableNumber("Drive/Auto/TurnKi", DriveConstants.turnKiAuto);
  private static final LoggedTunableNumber autoTurnKd =
      new LoggedTunableNumber("Drive/Auto/TurnKd", DriveConstants.turnKdAuto);

  /**
   * Thin adapter that lets the AutoBuilder-facing controller reference stay fixed forever while
   * the actual PPHolonomicDriveController underneath it gets swapped out live. AutoBuilder only
   * ever sees this wrapper, so it only needs to be configured once.
   */
  private static final class TunablePPController implements PathFollowingController {
    private volatile PPHolonomicDriveController delegate = buildController();

    private static PPHolonomicDriveController buildController() {
      return new PPHolonomicDriveController(
          new PIDConstants(autoDriveKp.get(), autoDriveKi.get(), autoDriveKd.get()),
          new PIDConstants(autoTurnKp.get(), autoTurnKi.get(), autoTurnKd.get()));
    }

    /** Rebuilds the inner controller from the current tunable values. */
    void refresh() {
      delegate = buildController();
    }

    @Override
    public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
      delegate.reset(currentPose, currentSpeeds);
    }

    @Override
    public ChassisSpeeds calculateRobotRelativeSpeeds(
        Pose2d currentPose, PathPlannerTrajectoryState targetState) {
      Logger.recordOutput("Drive/Auto/ActiveDriveKp", autoDriveKp.get());
      Logger.recordOutput("Drive/Auto/ActiveTurnKp", autoTurnKp.get());
      return delegate.calculateRobotRelativeSpeeds(currentPose, targetState);
    }

    @Override
    public boolean isHolonomic() {
      return delegate.isHolonomic();
    }
  }

  private final TunablePPController autoController = new TunablePPController();

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);
  
  private final PIDController choreoXController = new PIDController(3.5, 0.0, 0.0);
  private final PIDController choreoYController = new PIDController(3.5, 0.0, 0.0);
  private final PIDController choreoHeadingController = new PIDController(5, 0.0, 0.0);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    SparkOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner (only ever called once - see TunablePPController)
    //
    // NOTE: assigned to an explicitly-typed local first, rather than passed as an inline
    // lambda/method reference. AutoBuilder.configure() has two overloads that differ only in
    // this parameter's type (Consumer<ChassisSpeeds> vs BiConsumer<ChassisSpeeds,
    // DriveFeedforwards>). When every argument to configure() is an implicit lambda/method
    // reference, some compilers can't resolve which overload to use even when arity should
    // disambiguate it. Giving this one argument a concrete, already-resolved type sidesteps
    // that ambiguity entirely.
    java.util.function.BiConsumer<ChassisSpeeds, DriveFeedforwards> driveOutput =
        (speeds, feedforwards) -> runVelocity(speeds, feedforwards);
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        driveOutput,
        autoController,
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    choreoHeadingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Swap in a fresh inner controller if any tunable PID gain changed.
    // Only do this while disabled - swapping mid-auto would disrupt an active path follow.
    if (DriverStation.isDisabled()) {
      LoggedTunableNumber.ifChanged(
          hashCode(),
          (pid) -> autoController.refresh(),
          autoDriveKp,
          autoDriveKi,
          autoDriveKd,
          autoTurnKp,
          autoTurnKi,
          autoTurnKd);
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity, with no acceleration feedforward. Used by teleop and
   * stop() where there's no trajectory to pull per-module feedforwards from.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    runVelocity(speeds, DriveFeedforwards.zeros(4));
  }

  /**
   * Runs the drive at the desired velocity, using PathPlanner's per-module acceleration
   * feedforwards (kA) when following a trajectory.
   *
   * @param speeds Speeds in meters/sec
   * @param feedforwards Per-module acceleration/force feedforwards from PathPlanner, in FL, FR,
   *     BL, BR order. Pass {@code DriveFeedforwards.zeros(4)} if none are available (e.g. teleop).
   */
  public void runVelocity(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints (+ acceleration feedforward) to modules
    double[] accelsMPSSq = feedforwards.accelerationsMPSSq(); // FL, FR, BL, BR order
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i], accelsMPSSq[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /**
   * Choreo trajectory follower. Field-relative feedforward from the sample + feedback correction
   * from current pose error, converted to robot-relative before handing off to runVelocity().
   */
  public void followTrajectory(SwerveSample sample) {
      Pose2d pose = getPose();
  
      double xFF = sample.vx + choreoXController.calculate(pose.getX(), sample.x);
      double yFF = sample.vy + choreoYController.calculate(pose.getY(), sample.y);
      double headingFF =
          sample.omega
              + choreoHeadingController.calculate(pose.getRotation().getRadians(), sample.heading);
  
      ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(xFF, yFF, headingFF);
      ChassisSpeeds robotRelativeSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, pose.getRotation());
  
      Logger.recordOutput("Choreo/TargetPose", new Pose2d(sample.x, sample.y, Rotation2d.fromRadians(sample.heading)));
  
      runVelocity(robotRelativeSpeeds); // uses the no-feedforward overload we already built
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMetersPerSec;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return maxSpeedMetersPerSec / driveBaseRadius;
  }
}