package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/* 
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
*/

public class SwerveModule extends SubsystemBase{
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder turnEncoder;

    //private final CANCoder driveEncoder;
    private final AnalogInput absoluteEncoder;
    private PIDController turnPidController;
    private boolean absoluteEncoderReversed;
    private double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
        
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turnEncoder.setPositionConversionFactor(ModuleConstants.kTurnEncoderRot2Rad);
        turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurnEncoderRPM2RadPerSec);
        
        turnPidController = new PIDController(ModuleConstants.kPTurn, absoluteEncoderId, 0, 0);
        turnPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
        }

        public double getDrivePosition(){
            return driveEncoder.getPosition();
        }

        public double getTurnPosition() {
            return turnEncoder.getPosition();
        }
         
        public double getDriveVelocity() {
            return driveEncoder.getVelocity();
        }

        public double getTurningVelocity(){
            return turnEncoder.getVelocity();
        }

        public double getAbsoluteEncoderRad() {
            double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
            angle *= 2.0 * Math.PI;
            angle -= absoluteEncoderOffsetRad;
            return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
        }

        public void resetEncoders() {
            driveEncoder.setPosition(0);
            turnEncoder.setPosition(getAbsoluteEncoderRad());
        }

        public SwerveModuleState getState() {
            return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
        }

        public void setDesiredState(SwerveModuleState state) {
            if (Math.abs(state.speedMetersPerSecond) < 0.001) {
                stop();
                return;
            }
            state = SwerveModuleState.optimize(state, getState().angle);
            driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
            turnMotor.set(turnPidController.calculate(getTurnPosition(), state.angle.getRadians()));
            
        }

        public void stop() {
            driveMotor.set(0);
            turnMotor.set(0);
        }

}







/* 
    private final PIDController turningPIDController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless );
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless );

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
        
        //Change second parameter
        //driveEncoder = driveMotor.getEncoder();
        //turningEncoder = turningMotor.getEncoder();

        //Might give error when run
        driveEncoder = (CANCoder)driveMotor.getEncoder();
        turningEncoder = (CANCoder)turningMotor.getEncoder();

        driveEncoder.setPosition(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setPosition(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        //driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        //driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        //turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        //turningEncoder(ModuleConstants.kTurningEncoderRot2Meter);
        //turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPIDController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();

    }

    public double getDriverPosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition(){
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : -1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));

    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.01){
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }



}
*/