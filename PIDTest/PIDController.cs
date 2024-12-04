using System;

namespace PIDTest
{
    public class PIDController
    {
        private double m_ProportionalGain;
        private double m_IntegralGain;
        private double m_DerivativeGain;
        private double m_IntegralTerm = 0;
        private double m_PreviousError = 0;
        private double m_PreviousDerivative = 0;
        private double m_PreviousCommand = 0;
        private double m_PreviousSaturatedCommand = 0;

        private double m_DerivativeFilterConstant = 1.0;
        private double m_MaxRateOfChange = 10.0;

        public class PhysicalObject
        {
            public double Mass;      // Mass of the object
            public double Damping;   // Damping constant
            public double MaxForce;  // Maximum force the object can experience
            public double MinForce;  // Minimum force the object can experience
            public double TimeStep;  // Time step for simulation
            public double Acceleration;  // Current velocity of the object
            public double Velocity;  // Current position of the object
        }

        private PhysicalObject m_Object;

        public PIDController(double proportionalGain, double integralGain, double derivativeGain, PhysicalObject physicalObject)
        {
            m_ProportionalGain = proportionalGain;
            m_IntegralGain = integralGain;
            m_DerivativeGain = derivativeGain;
            m_Object = physicalObject;
        }

        public double ComputeControlSignal(double measurement, double setpoint)
        {
            double error = setpoint - measurement;
            double filteredDerivative;
            double controlSignal;
            double saturatedControlSignal;

            // Calculate the integral term with anti-windup
            m_IntegralTerm += m_IntegralGain * error * m_Object.TimeStep;

            // Calculate the filtered derivative term
            filteredDerivative = (error - m_PreviousError + m_DerivativeFilterConstant * m_PreviousDerivative) / (m_Object.TimeStep + m_DerivativeFilterConstant);
            m_PreviousError = error;
            m_PreviousDerivative = filteredDerivative;

            // Summing up the terms for the control signal
            controlSignal = m_ProportionalGain * error + m_IntegralTerm + m_DerivativeGain * filteredDerivative;

            // Saturate the control signal
            saturatedControlSignal = Math.Max(m_Object.MinForce, Math.Min(controlSignal, m_Object.MaxForce));

            // Anti-windup: Adjust integral term if the control signal is saturated
            if ((saturatedControlSignal == m_Object.MaxForce && m_IntegralTerm > 0) ||
                (saturatedControlSignal == m_Object.MinForce && m_IntegralTerm < 0))
            {
                m_IntegralTerm -= m_IntegralGain * error * m_Object.TimeStep;
            }

            // Rate limiting to avoid sudden jumps in the control signal
            if (saturatedControlSignal > m_PreviousSaturatedCommand + m_MaxRateOfChange * m_Object.TimeStep)
            {
                saturatedControlSignal = m_PreviousSaturatedCommand + m_MaxRateOfChange * m_Object.TimeStep;
            }
            else if (saturatedControlSignal < m_PreviousSaturatedCommand - m_MaxRateOfChange * m_Object.TimeStep)
            {
                saturatedControlSignal = m_PreviousSaturatedCommand - m_MaxRateOfChange * m_Object.TimeStep;
            }

            // Store previous values for the next iteration
            m_PreviousCommand = controlSignal;
            m_PreviousSaturatedCommand = saturatedControlSignal;

            return saturatedControlSignal;
        }

        public static double UpdateObjectState(PhysicalObject physicalObject, double appliedForce)
        {
            // Apply saturation to the force input
            double saturatedForce = Math.Max(physicalObject.MinForce, Math.Min(appliedForce, physicalObject.MaxForce));

            // Calculate the rate of change of velocity (acceleration)
            double acceleration = (saturatedForce - physicalObject.Damping * physicalObject.Acceleration) / physicalObject.Mass;

            // Update the velocity and position of the object
            physicalObject.Acceleration += acceleration * physicalObject.TimeStep;
            physicalObject.Velocity += physicalObject.Acceleration * physicalObject.TimeStep;

            return physicalObject.Velocity;
        }
    }
}
