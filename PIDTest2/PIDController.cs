using System;

namespace PIDTest2
{
    public class PIDController
    {
        private double Kp, Ki, Kd;
        private double integral = 0;
        private double previousError = 0;
        private double maxVelocity;

        public static double Clamp(double value, double min, double max) =>
            value < min ? min : value > max ? max : value;

        public PIDController(
            double Kp,
            double Ki,
            double Kd,
            double maxVelocity)
        {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
            this.maxVelocity = maxVelocity;
        }

        public double Compute(double currentVelocity, double setpoint, double timeStep)
        {
            double error = setpoint - currentVelocity;

            // Proportional term with reduced sensitivity
            double proportional = Kp * error * 0.5;

            // Integral term with tighter range
            integral += Ki * error * timeStep;
            integral = Clamp(integral, -2, 2);

            // Derivative term with damping
            double derivative = Kd * (error - previousError) / timeStep * 0.7;

            // Combine PID terms
            double pidOutput = proportional + integral + derivative;

            previousError = error;

            return pidOutput;
        }
    }
}