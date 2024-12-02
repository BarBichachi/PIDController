using System;
using System.Windows;
using System.Windows.Threading;

namespace PIDTest2
{
    public partial class MainWindow : Window
    {
        private PIDController pidController;
        private DispatcherTimer timer;
        private double currentVelocity = 0;
        private double targetSetpoint = 30;
        private double maxVelocity = 30;
        private double timeStep = 0.01;
        private double currentTime = 0;
        private double totalMovement = 0;

        public MainWindow()
        {
            InitializeComponent();

            pidController = new PIDController(
                Kp: 1.5,      // Reduced proportional gain
                Ki: 0.1,     // Low integral gain
                Kd: 0.3,      // Moderate derivative gain
                maxVelocity: 30.0
            );

            timer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(10)
            };
            timer.Tick += OnSimulationTick;
        }

        private void StartSimulation(object sender, RoutedEventArgs e)
        {
            currentVelocity = 0;
            currentTime = 0;
            totalMovement = 0;
            SetpointTextBlock.Text = $"Setpoint: {targetSetpoint:F2} d/s";
            timer.Start();
        }

        private void OnSimulationTick(object sender, EventArgs e)
        {
            double controlSignal = pidController.Compute(currentVelocity, targetSetpoint, timeStep);

            currentVelocity += controlSignal * timeStep;

            VelocityTextBlock.Text = $"Velocity: {currentVelocity:F2} d/s";

            currentTime += timeStep;
            totalMovement += currentVelocity * timeStep;
            DegreeTextBlock.Text = $"Total Movement: {totalMovement:F2} degrees";
        }
    }
}