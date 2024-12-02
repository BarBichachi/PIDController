using System;

namespace PIDTest
{
    public class PIDController
    {
        private double Kp, Ki, Kd;
        private double integral = 0;
        private double previousError = 0;
        private double previousDerivative = 0;
        private double previousCommand = 0;
        private double previousSaturatedCommand = 0;

        private double T_C = 1.0; // Derivative filter constant
        private double maxRate = 10.0; // Max rate of change

        public class Object
        {
            public double m;    // Mass
            public double k;    // Damping constant
            public double F_max; // Max force
            public double F_min; // Min force
            public double T;    // Time step
            public double v;    // Velocity
            public double z;    // Position
        }

        private Object obj;

        public PIDController(double Kp, double Ki, double Kd, Object obj)
        {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
            this.obj = obj;
        }

        public double PIDStep(double measurement, double setpoint)
        {
            double error = setpoint - measurement;
            double derivativeFiltered;
            double command;
            double commandSaturated;

            // Integral term calculation with anti-windup
            integral += Ki * error * obj.T;

            // Derivative term calculation (filtered)
            derivativeFiltered = (error - previousError + T_C * previousDerivative) / (obj.T + T_C);
            previousError = error;
            previousDerivative = derivativeFiltered;

            // Summing the terms
            command = Kp * error + integral + Kd * derivativeFiltered;

            // Saturation
            commandSaturated = Math.Max(obj.F_min, Math.Min(command, obj.F_max));

            // Anti-windup logic
            if ((commandSaturated == obj.F_max && integral > 0) ||
                (commandSaturated == obj.F_min && integral < 0))
            {
                // Reset integral to prevent excessive accumulation
                integral -= Ki * error * obj.T; // Reduce the accumulated integral
            }

            // Rate limiting
            if (commandSaturated > previousSaturatedCommand + maxRate * obj.T)
            {
                commandSaturated = previousSaturatedCommand + maxRate * obj.T;
            }
            else if (commandSaturated < previousSaturatedCommand - maxRate * obj.T)
            {
                commandSaturated = previousSaturatedCommand - maxRate * obj.T;
            }

            // Remember previous values for next iteration
            previousCommand = command;
            previousSaturatedCommand = commandSaturated;

            return commandSaturated;
        }
        public static double ObjectStep(Object obj, double F)
        {
            // Apply saturation to the input force
            double F_sat = F > obj.F_max ? obj.F_max : (F < obj.F_min ? obj.F_min : F);

            // Calculate the derivative dv/dt using force and object properties
            double dv_dt = (F_sat - obj.k * obj.v) / obj.m;

            // Update the velocity and position of the object
            obj.v += dv_dt * obj.T;
            obj.z += obj.v * obj.T;

            return obj.z;
        }
    }
}