package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDGains;

import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {
    
        // NetworkTable publishers
        private final NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("intake");
    
        private final SparkMax intakeMotor;
        private final SparkMaxConfig intakeMotorConfig;
    
        private final RelativeEncoder intakeEncoder;
        private final SparkClosedLoopController intakeController;
        private final PIDGains intakePIDGains = new PIDGains(1.0, 0, 0);

        private double filteredCurrent = 0;
        private double currentFilterConstant = 0.1;
    
    public IntakeSubsystem() { // CONSTRUCTION
        intakeMotor = new SparkMax(INTAKE_MOTOR_ID, SparkMax.MotorType.kBrushless);
        intakeMotorConfig = new SparkMaxConfig();

        intakeEncoder = intakeMotor.getEncoder();
        intakeEncoder.setPosition(0);
        intakeController = intakeMotor.getClosedLoopController();

        configMotors();
    }

    public void teleopInit() {
        setIntakeDutyCycle(0);
    }

    @Override
    public void periodic() {
        filterCurrent();
        updateTelemetry();
    }

    private void filterCurrent() {
        filteredCurrent = filteredCurrent * (1 - currentFilterConstant) + intakeMotor.getOutputCurrent() * currentFilterConstant;
    }

    private void updateTelemetry() {
    }

    @SuppressWarnings("removal")
    private void configMotors() {

        intakeMotorConfig
            .inverted(false)
            .smartCurrentLimit(40);

        intakeMotorConfig.closedLoop
            .pid(intakePIDGains.p, intakePIDGains.i, intakePIDGains.d);

        intakeMotor
            .configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setIntakeDutyCycle(double dutyCycle) {
        intakeController.setSetpoint(dutyCycle, ControlType.kDutyCycle);
    }

    public void setIntakePosition(double position) {
        intakeController.setSetpoint(position, ControlType.kPosition);
    }

    public void holdIntake() {
        setIntakePosition(intakeEncoder.getPosition());
    }
    
    public void stopIntake() {
        setIntakeDutyCycle(0);
    }

    public Command runIntakeCommand() {
        return Commands.runEnd(() -> setIntakeDutyCycle(0.1), () -> stopIntake());
    }
}