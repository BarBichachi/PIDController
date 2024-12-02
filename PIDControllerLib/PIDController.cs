using System;
namespace PIDControllerLib
{
    public class PIDController
    {
        // PID gains
        public double ProportionalGain { get; set; }
        public double IntegralGain { get; set; }
        public double DerivativeGain { get; set; }

        // Target and current values
        public double Setpoint { get; set; }
        public double CurrentPosition { get; private set; }
        public double Velocity { get; private set; }

        // Time step
        public double DeltaTime { get; set; }

        // Precision tolerance
        public double Tolerance { get; set; }

        // Maximum velocity and acceleration (degrees per second)
        public double MaxVelocity { get; set; }
        public double MaxAcceleration { get; set; }

        // Internal state
        private double m_PreviousError;
        private double m_IntegralSum;
        private double m_PreviousVelocity;

        // Event to signal target reached
        public event EventHandler OnTargetReached;

        public PIDController(double i_PGain, double i_IGain, double i_DGain)
        {
            ProportionalGain = i_PGain;
            IntegralGain = i_IGain;
            DerivativeGain = i_DGain;

            // Initialize the event to avoid nullable warning
            OnTargetReached += delegate { };
        }

        /// <summary>
        /// Updates the current position based on PID logic with gradual acceleration and precise approach
        /// </summary>
        public void Update()
        {
            // Normalize the angles to the 0-360 range
            Setpoint = NormalizeAngle(Setpoint);
            CurrentPosition = NormalizeAngle(CurrentPosition);

            // Calculate the error, considering the shortest path between the current position and setpoint
            double error = ShortestAngularDistance(CurrentPosition, Setpoint);

            // Proportional term with adaptive gain
            double adaptiveProportionalGain = ProportionalGain;

            // Adjust gain based on distance from setpoint
            if (Math.Abs(error) < 10)
            {
                adaptiveProportionalGain *= 1.5;
            }

            // Proportional term
            double proportional = adaptiveProportionalGain * error;

            // Integral term
            m_IntegralSum += error * DeltaTime;
            m_IntegralSum = Math.Max(-10, Math.Min(m_IntegralSum, 10));
            double integral = IntegralGain * m_IntegralSum;

            // Derivative term
            double derivative = DerivativeGain * (error - m_PreviousError) / DeltaTime;

            // Calculate the target velocity
            double targetVelocity = proportional + integral + derivative;

            // Gradual acceleration
            double velocityChange = targetVelocity - m_PreviousVelocity;

            // Limit acceleration
            velocityChange = Math.Max(-MaxAcceleration * DeltaTime, Math.Min(MaxAcceleration * DeltaTime, velocityChange));

            // Update velocity
            Velocity = m_PreviousVelocity + velocityChange;

            // Limit overall velocity
            Velocity = Math.Max(-MaxVelocity, Math.Min(MaxVelocity, Velocity));

            // New velocity reduction strategy
            double absoluteError = Math.Abs(error);

            if (absoluteError <= 2)  // When within 2 degrees of the setpoint
            {
                // Exponential decay factor based on the distance to the setpoint
                double reductionFactor = Math.Exp(-absoluteError / 2);  // Adjust divisor for smoother or faster decay

                // Apply the reduction factor
                Velocity *= reductionFactor;

                // Clamp the velocity to ensure it doesn't overshoot
                Velocity = Clamp(Velocity, -MaxVelocity, MaxVelocity);

                // Ensure the velocity has the correct sign
                Velocity = error > 0 ? Velocity : -Velocity;
            }

            // Check if very close to setpoint and apply stop conditions
            if (Math.Abs(error) <= Tolerance)
            {
                Velocity *= 0.1; // Reduce velocity to prevent overshoot
                if (Math.Abs(Velocity) < 0.01) // Stop if velocity is very small
                {
                    Velocity = 0;
                    CurrentPosition = Setpoint;
                    OnTargetReached?.Invoke(this, EventArgs.Empty);
                    return;
                }
            }

            // Update the current position based on velocity
            CurrentPosition += Velocity * DeltaTime;

            // Normalize position to ensure it stays within 0-360 degrees
            CurrentPosition = NormalizeAngle(CurrentPosition);

            // Save current state for next iteration
            m_PreviousError = error;
            m_PreviousVelocity = Velocity;
        }

        public void Reset()
        {
            m_IntegralSum = 0;
            m_PreviousError = 0;
            m_PreviousVelocity = 0;
            Velocity = 0;
            CurrentPosition = 0;
        }

        private static double NormalizeAngle(double angle) => (angle % 360 + 360) % 360;

        private double ShortestAngularDistance(double current, double target)
        {
            double error = target - current;

            // Adjust for shortest path around circle
            if (error > 180) error -= 360;
            if (error < -180) error += 360;

            return error;
        }

        // Replace Math.Clamp() with this method
        private static double Clamp(double value, double min, double max)
        {
            return Math.Max(min, Math.Min(max, value));
        }

    }
}