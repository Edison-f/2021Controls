#include "Example.h"

Example::Example(frc::Joystick *joystick) {

    m_joystick = joystick;
    m_spark_1 = new rev::CANSparkMax(0, rev::CANSparkMax::MotorType::kBrushless);
    m_spark_PID_1 = new rev::CANPIDController(m_spark_1->GetPIDController());
    m_spark_encoder_1 = new rev::CANEncoder(m_spark_1->GetEncoder());
    m_talonfx_1 = new WPI_TalonFX(0);

    setpoint = m_spark_encoder_1->GetPosition();

    ConfigureSpark(m_spark_1, m_spark_PID_1, 
        kP, kI, kD, kIz, kFF, minOut, maxOut, rev::CANSparkMax::IdleMode::kCoast);
    ConfigureTalon(m_talonfx_1, 0, 0, 10, 
        false, kFF, kP, kI, kD, ctre::phoenix::motorcontrol::NeutralMode::Coast);

}

/**
 * 
 * Runs motors using percent output from -1 to 1 inclusive
 * 
 */
void Example::Run() { 

    frc::SmartDashboard::PutNumber("Joystick value" , m_joystick->GetRawAxis(1));

    m_spark_1->Set(m_joystick->GetRawAxis(0));
    m_talonfx_1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_joystick->GetRawAxis(1));

}


/**
 * 
 * Sets the desired velocity of both the spark and the falcon
 * 
 */
void Example::PIDRunVelocity() {

    frc::SmartDashboard::PutNumber("Joystick value" , m_joystick->GetRawAxis(1));
    
    m_spark_PID_1->SetReference(668, rev::ControlType::kVelocity);
    m_talonfx_1->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 668);

}

/**
 * 
 * Sets a desired position which is incremented by 668 units each time it is run
 * 
 */
void Example::PIDRunPosition() {
    
    frc::SmartDashboard::PutNumber("Joystick value" , m_joystick->GetRawAxis(1));
    frc::SmartDashboard::PutNumber("Setpoint", setpoint);

    setpoint += 668;

    m_spark_PID_1->SetReference(setpoint, rev::ControlType::kPosition);
    m_talonfx_1->Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
}

/**
 * 
 * Configures a spark and its associated PID
 * 
 */
void Example::ConfigureSpark(rev::CANSparkMax *spark, rev::CANPIDController *PIDController, 
        double kP, double kI, double kD, double kIz, double kFF, double minOut, 
        double maxOut, rev::CANSparkMax::IdleMode idleMode) {
        PIDController->SetP(kP);
        PIDController->SetI(kI);
        PIDController->SetD(kD);
        PIDController->SetIZone(kIz);
        PIDController->SetFF(kFF);
        PIDController->SetOutputRange(minOut, maxOut);

        spark->SetIdleMode(idleMode);

        spark->BurnFlash();
}


/**
 * 
 * Configures a falcon and its associated PID
 * 
 */
void Example::ConfigureTalon(WPI_TalonFX *talon, int pidSlot, int pidType, int timeoutMs, bool inverted,  
        int kF, int kP, int kI, int kD, NeutralMode brakeMode) {
        talon->ConfigFactoryDefault();
        talon->SetInverted(inverted);
        talon->Config_kF(pidSlot, kF, timeoutMs);
        talon->Config_kP(pidSlot, kP, timeoutMs);
        talon->Config_kI(pidSlot, kI, timeoutMs);
        talon->Config_kD(pidSlot, kD, timeoutMs);
        talon->SetNeutralMode(brakeMode);

        talon->SetSelectedSensorPosition(0, pidType, timeoutMs);
}