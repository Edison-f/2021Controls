#include <iostream>

#include <ctre/Phoenix.h> // http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/Phoenix-latest.json
#include <rev/CANSparkMax.h> // https://www.revrobotics.com/content/sw/max/sdk/REVRobotics.json
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Example {

    public:

    Example(frc::Joystick *joystick);
    void Run();
    void PIDRunVelocity();
    void PIDRunPosition();

    double kP = 10.8; //P
    double kI = 0; //I
    double kD = 0; //D
    double kIz = 0; //I zone - Range the absolute error needs to be for I to be used
    double kFF = 0; //Feed forward
    double minOut = -1.0; //Minimum output
    double maxOut = 1.0; //Maximum output
    
    double setpoint;
    
    private:

    frc::Joystick *m_joystick;

    rev::CANSparkMax *m_spark_1;
    rev::CANPIDController *m_spark_PID_1;
    rev::CANEncoder *m_spark_encoder_1;

    WPI_TalonFX *m_talonfx_1;

    void ConfigureSpark(rev::CANSparkMax *spark, rev::CANPIDController *PIDController, double kP, double kI, double kD, 
        double kIz, double kFF, double minOut, double maxOut, rev::CANSparkMax::IdleMode idleMode);
    void ConfigureTalon(WPI_TalonFX *talon, int pidSlot, int pidType, int timeoutMs, bool inverted,  
        int kF, int kP, int kI, int kD, NeutralMode brakeMode);


};