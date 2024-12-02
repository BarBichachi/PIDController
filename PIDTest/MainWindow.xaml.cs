using System;
using System.Windows;
using System.Windows.Threading;

namespace PIDTest
{
    public partial class MainWindow : Window
    {
        private PIDController pid1;
        private PIDController.Object obj1;
        private DispatcherTimer timer;
        private double t = 0;
        private int i = 0;
        private double command1 = 0;
        private double stp1 = 30;  // Initial setpoint of 30 degrees
        private double z1 = 0;

        public MainWindow()
        {
            InitializeComponent();

            // Initialize PIDController and Object
            obj1 = new PIDController.Object { m = 10, k = 0.5f, F_max = 5, F_min = -5, T = 0.1f, v = 0, z = 0 };
            pid1 = new PIDController(1.5, 0.0001, 5, obj1);

            // Set up a timer to update the UI periodically
            timer = new DispatcherTimer();
            timer.Interval = TimeSpan.FromMilliseconds(1); // Update every 1 ms
            timer.Tick += Timer_Tick;
        }

        private void StartSimulation(object sender, RoutedEventArgs e)
        {
            timer.Start();
        }

        private double previousCommand = 0; // Add this field to track the previous command

        private void Timer_Tick(object sender, EventArgs e)
        {
            // Compute control output
            double targetCommand = pid1.PIDStep(z1, stp1);

            // Apply gradual change to the command
            double forceStep = obj1.F_max * 0.1; // Adjust this value for smoother transitions
            if (Math.Abs(targetCommand - previousCommand) > forceStep)
            {
                // Gradually adjust the command
                targetCommand = previousCommand + Math.Sign(targetCommand - previousCommand) * forceStep;
            }

            // Saturate the command
            command1 = Math.Max(Math.Min(targetCommand, obj1.F_max), obj1.F_min);

            // Update object dynamics
            z1 = PIDController.ObjectStep(obj1, command1);

            // Update the UI
            VelocityTextBlock.Text = $"{obj1.v:F2} d/s";
            DegreeTextBlock.Text = $"{obj1.z:F2} d/s";

            // Increment time
            t += obj1.T;
            i++;

            // Check if we need to stop
            if (Math.Abs(z1 - 30) < 0.001 && stp1 == 30)
            {
                timer.Stop();
                VelocityTextBlock.Text = $"{0:F2} d/s";
            }

            // Store the previous command for the next iteration
            previousCommand = command1;
        }

    }
}
