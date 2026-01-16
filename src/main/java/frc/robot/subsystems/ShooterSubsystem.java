package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDGains;

import static frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase { 
    
        // NetworkTable publishers
        private final NetworkTable shooter1Table = NetworkTableInstance.getDefault().getTable("shooter1");
        private final DoublePublisher speedPublisher = shooter1Table.getDoubleTopic("speed").publish();
    
        private final SparkMax shooter1Motor;
        private final SparkMaxConfig shooter1MotorConfig;

        private double speed = 0;
    
        private final RelativeEncoder shooter1Encoder;
        private final SparkClosedLoopController shooter1Controller;
        private final PIDGains shooter1PIDGains = new PIDGains(1.0, 0, 0);

        private final SparkMax shooter2Motor;
        private final SparkMaxConfig shooter2MotorConfig;
    
        private final RelativeEncoder shooter2Encoder;
        private final SparkClosedLoopController shooter2Controller;
        private final PIDGains shooter2PIDGains = new PIDGains(1.0, 0, 0);

        private double filteredCurrent = 0;
        private double currentFilterConstant = 0.1;
    
    public ShooterSubsystem() { // CONSTRUCTION
        shooter1Motor = new SparkMax(SHOOTER_MOTOR_1_ID, SparkMax.MotorType.kBrushless);
        shooter1MotorConfig = new SparkMaxConfig();

        shooter1Encoder = shooter1Motor.getEncoder();
        shooter1Encoder.setPosition(0);
        shooter1Controller = shooter1Motor.getClosedLoopController();

        shooter2Motor = new SparkMax(SHOOTER_MOTOR_2_ID, SparkMax.MotorType.kBrushless);
        shooter2MotorConfig = new SparkMaxConfig();

        shooter2Encoder = shooter2Motor.getEncoder();
        shooter2Encoder.setPosition(0);
        shooter2Controller = shooter2Motor.getClosedLoopController();

        configMotors();

    }

    public void teleopInit() {
        setShooterDutyCycle(0);
        SmartDashboard.putNumber("speed", speed);
    }

    @Override
    public void periodic() {
        filterCurrent();
        updateTelemetry();
    }

    private void filterCurrent() {
        filteredCurrent = filteredCurrent * (1 - currentFilterConstant) + shooter1Motor.getOutputCurrent() * currentFilterConstant;
    }

    private void updateTelemetry() {
        speedPublisher.set(speed);
    }

    @SuppressWarnings("removal")
    private void configMotors() {

        shooter1MotorConfig
            .inverted(false)
            .smartCurrentLimit(60);

        shooter1MotorConfig.closedLoop
            .pid(shooter1PIDGains.p, shooter1PIDGains.i, shooter1PIDGains.d);

        shooter1Motor
            .configure(shooter1MotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        shooter2MotorConfig
            .inverted(true);
        
        shooter2MotorConfig.closedLoop
            .pid(shooter1PIDGains.p, shooter1PIDGains.i, shooter1PIDGains.d);
        
        shooter2Motor
            .configure(shooter2MotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setShooterDutyCycle(double dutyCycle) {
        speed = dutyCycle;
        shooter1Controller.setSetpoint(dutyCycle, ControlType.kDutyCycle);
        shooter2Controller.setSetpoint(dutyCycle, ControlType.kDutyCycle);
    }

    public void setShooterPosition(double position) {
        shooter1Controller.setSetpoint(position, ControlType.kPosition);
    }

    public void holdShooter() {
        setShooterPosition(shooter1Encoder.getPosition());
    }
    
    public void stopShooter() {
        speed = 0;
        setShooterDutyCycle(speed);
    }

    public Command runShooterCommand() {
        return Commands.runEnd(() -> setShooterDutyCycle(0.2), () -> stopShooter());
    }
}