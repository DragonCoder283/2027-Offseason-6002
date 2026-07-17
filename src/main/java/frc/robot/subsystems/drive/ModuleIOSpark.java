// Copyright (c) 2021-2026 Littleton Robotics 
// http://github.com 
// 
// Use of this source code is governed by a BSD 
// license that can be found in the LICENSE file 
// at the root directory of this project. 

package frc.robot.subsystems.drive; 

import static frc.robot.subsystems.drive.DriveConstants.*; 
import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder; 
import com.revrobotics.PersistMode; 
import com.revrobotics.RelativeEncoder; 
import com.revrobotics.ResetMode; 
import com.revrobotics.spark.ClosedLoopSlot; 
import com.revrobotics.spark.FeedbackSensor; 
import com.revrobotics.spark.SparkBase; 
import com.revrobotics.spark.SparkBase.ControlType; 
import com.revrobotics.spark.SparkClosedLoopController; 
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits; 
import com.revrobotics.spark.SparkFlex; 
import com.revrobotics.spark.SparkLowLevel.MotorType; 
import com.revrobotics.spark.SparkMax; 
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode; 
import com.revrobotics.spark.config.SparkFlexConfig; 
import com.revrobotics.spark.config.SparkMaxConfig; 

import edu.wpi.first.math.MathUtil; 
import edu.wpi.first.math.filter.Debouncer; 
import edu.wpi.first.math.geometry.Rotation2d; 
import java.util.Queue; 
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger; 

/** 
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller, 
 * and CAN bus CTRE CANcoder. 
 */ 
public class ModuleIOSpark implements ModuleIO { 
    private final Rotation2d zeroRotation; 

    // Hardware objects 
    private final SparkBase driveSpark;
    private final boolean driveInverted; 
    private final SparkBase turnSpark; 
    private final RelativeEncoder driveEncoder; 
    private final RelativeEncoder turnEncoder; // Changed from AbsoluteEncoder to RelativeEncoder
    double absolutePosition;

    // Closed loop controllers 
    private final SparkClosedLoopController driveController; 
    private final SparkClosedLoopController turnController; 

    // Queue inputs from odometry thread 
    private final Queue<Double> timestampQueue; 
    private final Queue<Double> drivePositionQueue; 
    private final Queue<Double> turnPositionQueue; 

    // Connection debouncers 
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling); 
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling); 

    public ModuleIOSpark(int module) { 
        zeroRotation = switch (module) { 
            case 0 -> frontLeftZeroRotation; 
            case 1 -> frontRightZeroRotation; 
            case 2 -> backLeftZeroRotation; 
            case 3 -> backRightZeroRotation; 
            default -> Rotation2d.kZero; 
        }; 

        driveSpark = new SparkFlex( 
            switch (module) { 
                case 0 -> frontLeftDriveCanId; 
                case 1 -> frontRightDriveCanId; 
                case 2 -> backLeftDriveCanId; 
                case 3 -> backRightDriveCanId; 
                default -> 0; 
            }, MotorType.kBrushless); 
        
        driveInverted = 
            switch (module) { 
                case 0 -> false; 
                case 1 -> true; 
                case 2 -> false; 
                case 3 -> true; 
                default -> false; 
            };

        turnSpark = new SparkMax( 
            switch (module) { 
                case 0 -> frontLeftTurnCanId; 
                case 1 -> frontRightTurnCanId; 
                case 2 -> backLeftTurnCanId; 
                case 3 -> backRightTurnCanId; 
                default -> 0; 
            }, MotorType.kBrushless); 

        driveEncoder = driveSpark.getEncoder(); 
        turnEncoder = turnSpark.getEncoder(); // Changed to getEncoder() instead of getAbsoluteEncoder()
        
        driveController = driveSpark.getClosedLoopController(); 
        turnController = turnSpark.getClosedLoopController(); 

        // Configure drive motor 
        var driveConfig = new SparkFlexConfig(); 
        driveConfig 
            .idleMode(IdleMode.kCoast) 
            .smartCurrentLimit(driveMotorCurrentLimit) 
            .voltageCompensation(12.0)
            .inverted(driveInverted); 

        driveConfig
            .encoder 
            .positionConversionFactor(driveEncoderPositionFactor) 
            .velocityConversionFactor(driveEncoderVelocityFactor) 
            .uvwMeasurementPeriod(10) 
            .uvwAverageDepth(2); 

        driveConfig 
            .closedLoop 
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder) 
            .pid(driveKp, 0.0, driveKd); 

        driveConfig 
            .signals 
            .primaryEncoderPositionAlwaysOn(true) 
            .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency)) 
            .primaryEncoderVelocityAlwaysOn(true) 
            .primaryEncoderVelocityPeriodMs(20) 
            .appliedOutputPeriodMs(20) 
            .busVoltagePeriodMs(20) 
            .outputCurrentPeriodMs(20); 

        tryUntilOk( 
            driveSpark, 5, () -> driveSpark.configure( 
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)); 
        tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0)); 

        // Configure turn motor 
        var turnConfig = new SparkMaxConfig(); 
        turnConfig 
            .inverted(turnInverted) 
            .idleMode(IdleMode.kCoast) 
            .smartCurrentLimit(turnMotorCurrentLimit) 
            .voltageCompensation(12.0); 

        // Target the internal relative encoder configuration instead of absoluteEncoder
        turnConfig 
            .encoder 
            .positionConversionFactor(turnEncoderPositionFactor) 
            .velocityConversionFactor(turnEncoderVelocityFactor); 

        // Use the internal primary encoder for the PID control loop
        turnConfig 
            .closedLoop 
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder) 
            .positionWrappingEnabled(true) 
            .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput) 
            .pid(turnKp, 0.0, turnKd); 

        turnConfig 
            .signals 
            .primaryEncoderPositionAlwaysOn(true) // Changed to target primary encoder instead of absolute
            .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency)) 
            .primaryEncoderVelocityAlwaysOn(true) 
            .primaryEncoderVelocityPeriodMs(20) 
            .appliedOutputPeriodMs(20) 
            .busVoltagePeriodMs(20) 
            .outputCurrentPeriodMs(20); 

        tryUntilOk( 
            turnSpark, 5, () -> turnSpark.configure( 
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)); 

        // Instantiate the CAN Bus CANcoder
        var turnEncoderCTRE = new CANcoder( 
            switch (module) { 
                case 0 -> 31; 
                case 1 -> 32; 
                case 2 -> 33; 
                case 3 -> 34; 
                default -> 0; 
            }); 

        var turnEncoderOffsets = switch (module) { 
            case 0 -> -0.3408203125; 
            case 1 -> 0.419921875; 
            case 2 -> -0.050537109375; 
            case 3 -> -0.173583984375; 
            default -> 0; 
        };

        CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.MagnetOffset = turnEncoderOffsets;
        turnEncoderCTRE.getConfigurator().apply(config);

        // Seed the high-speed internal encoder with the absolute CANcoder position on boot
        // Get position returns rotations. Ensure your turnEncoderPositionFactor matches the expected units (usually Radians for AdvantageKit setups)
        turnEncoder.setPosition(0);
        double absolutePosition = turnEncoderCTRE.getAbsolutePosition().getValueAsDouble();
        tryUntilOk(turnSpark, 5, () -> turnEncoder.setPosition(Rotation2d.fromRotations(absolutePosition).getRadians()));

        // Create odometry queues 
        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue(); 
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition); 
        turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition); 
    } 

    @Override 
    public void updateInputs(ModuleIOInputs inputs) { 
        Logger.recordOutput("Encoders/RelativeEncoder", turnEncoder.getPosition());
        Logger.recordOutput("Encoders/AbsoluteEncoder", absolutePosition);
        // Update drive inputs 
        sparkStickyFault = false; 
        ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value); 
        ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value); 
        ifOk( 
            driveSpark, new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage}, 
            (values) -> inputs.driveAppliedVolts = values[0] * values[1]); 
        ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value); 
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault); 

        // Update turn inputs 
        sparkStickyFault = false; 
        ifOk( 
            turnSpark, turnEncoder::getPosition, 
            (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation)); 
        ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value); 
        ifOk( 
            turnSpark, new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage}, 
            (values) -> inputs.turnAppliedVolts = values[0] * values[1]); 
        ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value); 
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault); 

        // Update odometry inputs 
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray(); 
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray(); 
        inputs.odometryTurnPositions = turnPositionQueue.stream() 
            .map((Double value) -> new Rotation2d(value).minus(zeroRotation)) 
            .toArray(Rotation2d[]::new); 

        timestampQueue.clear(); 
        drivePositionQueue.clear(); 
        turnPositionQueue.clear(); 
    } 

    @Override 
    public void setDriveOpenLoop(double output) { 
        driveSpark.setVoltage(output); 
    } 

    @Override 
    public void setTurnOpenLoop(double output) { 
        turnSpark.setVoltage(output); 
    } 

    @Override 
    public void setDriveVelocity(double velocityRadPerSec) { 
        double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec; 
        driveController.setSetpoint( 
            velocityRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts, ArbFFUnits.kVoltage);}
            @Override
            public void setTurnPosition(Rotation2d rotation) {double setpoint = MathUtil.inputModulus(rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput);turnController.setSetpoint(setpoint, ControlType.kPosition);}}