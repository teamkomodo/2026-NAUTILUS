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

import org.littletonrobotics.junction.wpilog.WPILOGWriter.AdvantageScopeOpenBehavior;

public class IntakeSubsystem extends SubsystemBase {
    
        // NetworkTable publishers
        private final NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("intake");
        private final DoublePublisher intakeeSpeedPublisher = intakeTable.getDoubleTopic("speed").publish();
    
        private final SparkMax intakeMotor;
        private final SparkMaxConfig intakeMotorConfig;

        private double p = 1.0;
        private double i = 0.0;
        private double d = 0.0;
        private double speed = 0.0;
        private boolean cwDirection = true;
    
        private final RelativeEncoder intakeEncoder;
        private final SparkClosedLoopController intakeController;
        private PIDGains intakePIDGains = new PIDGains(p, i, d);

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
        setIntakeDutyCycle(speed);
    }

    private void filterCurrent() {
        filteredCurrent = filteredCurrent * (1 - currentFilterConstant) + intakeMotor.getOutputCurrent() * currentFilterConstant;
    }

    private void updateTelemetry() {
    
        SmartDashboard.putNumber("PID P", p);
        SmartDashboard.putNumber("PID I", i);
        SmartDashboard.putNumber("PID D", d);

        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putBoolean("Direction", cwDirection);
    }

    @SuppressWarnings("removal")
    private void configMotors() {

        intakeMotorConfig
            .inverted(cwDirection)
            .smartCurrentLimit(40);

        intakeMotorConfig.closedLoop
            .pid(intakePIDGains.p, intakePIDGains.i, intakePIDGains.d);

        intakeMotor
            .configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setIntakeDutyCycle(double dutyCycle) {
        speed = dutyCycle;
        intakeController.setSetpoint(dutyCycle, ControlType.kDutyCycle);
    }

    public void setIntakePosition(double position) {
        intakeController.setSetpoint(position, ControlType.kPosition);
    }

    public void holdIntake() {
        setIntakePosition(intakeEncoder.getPosition());
    }
    
    public void stopIntake() {
        speed = 0;
        setIntakeDutyCycle(0);
    }

    public Command runIntakeCommand() {
        return Commands.runEnd(() -> setIntakeDutyCycle(0.1), () -> stopIntake());
    }

    public void updateMotorValues() {
        intakePIDGains = new PIDGains(p, i, d);

        intakeMotorConfig
            .inverted(cwDirection);

        intakeMotorConfig.closedLoop
            .pid(intakePIDGains.p, intakePIDGains.i, intakePIDGains.d);

        intakeMotor
            .configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}