using AwesomeAssertions;
using Gehtsoft.Measurements;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Xunit;

namespace BallisticCalculator.Test.Calculator
{
    public class TrajectoryCalculatorTest
    {
        [Theory]
        [InlineData(0.365, DragTableId.G1, 2600, VelocityUnit.FeetPerSecond, 100, DistanceUnit.Yard, 5.674, AngularUnit.MOA, 5e-2)]
        [InlineData(0.365, DragTableId.G1, 2600, VelocityUnit.FeetPerSecond, 25, DistanceUnit.Yard, 12.84, AngularUnit.MOA, 5e-2)]
        [InlineData(0.365, DragTableId.G1, 2600, VelocityUnit.FeetPerSecond, 375, DistanceUnit.Yard, 12.78, AngularUnit.MOA, 5e-2)]
        [InlineData(0.47, DragTableId.G7, 725, VelocityUnit.MetersPerSecond, 200, DistanceUnit.Meter, 8.171, AngularUnit.MOA, 5e-2)]
        public void Zero1(double ballisticCoefficient, DragTableId ballisticTable, double muzzleVelocity, VelocityUnit velocityUnit, double zeroDistance, DistanceUnit distanceUnit, double sightAngle, AngularUnit sightAngleUnit, double sightAngleAccuracy)
        {
            Ammunition ammunition = new Ammunition(
                weight: new Measurement<WeightUnit>(69, WeightUnit.Grain),
                muzzleVelocity: new Measurement<VelocityUnit>(muzzleVelocity, velocityUnit),
                ballisticCoefficient: new BallisticCoefficient(ballisticCoefficient, ballisticTable)
                );

            Rifle rifle = new Rifle(
                sight: new Sight(sightHeight: new Measurement<DistanceUnit>(3.2, DistanceUnit.Inch), Measurement<AngularUnit>.ZERO, Measurement<AngularUnit>.ZERO),
                zero: new ZeroingParameters(distance: new Measurement<DistanceUnit>(zeroDistance, distanceUnit), ammunition: null, atmosphere: null));

            Atmosphere atmosphere = new Atmosphere();       //default atmosphere

            var sightAngle1 = (new TrajectoryCalculator()).SightAngle(ammunition, rifle, atmosphere);
            sightAngle1.In(sightAngleUnit).Should().BeApproximately(sightAngle, sightAngleAccuracy);
        }

        [Theory]
        [InlineData(null, 5.489)]
        [InlineData(0.0, 5.489)]
        [InlineData(-2.0, 3.579)]
        [InlineData(-0.5, 5.012)]
        [InlineData(1.0, 6.444)]
        public void Zero_WithOffset(double? offset, double sightAngle)
        {
            Ammunition ammunition = new Ammunition(
                weight: new Measurement<WeightUnit>(69, WeightUnit.Grain),
                muzzleVelocity: new Measurement<VelocityUnit>(2600, VelocityUnit.FeetPerSecond),
                ballisticCoefficient: new BallisticCoefficient(0.355, DragTableId.G1));

            Rifle rifle = new Rifle(
                sight: new Sight(sightHeight: new Measurement<DistanceUnit>(3, DistanceUnit.Inch), Measurement<AngularUnit>.ZERO, Measurement<AngularUnit>.ZERO),
                zero: new ZeroingParameters(distance: new Measurement<DistanceUnit>(100, DistanceUnit.Yard), ammunition: null, atmosphere: null));

            if (offset != null)
                rifle.Zero.VerticalOffset = DistanceUnit.Inch.New(offset.Value);

            Atmosphere atmosphere = new Atmosphere();       //default atmosphere

            var sightAngle1 = (new TrajectoryCalculator()).SightAngle(ammunition, rifle, atmosphere);
            sightAngle1.In(AngularUnit.MOA).Should().BeApproximately(sightAngle, 5e-2);
        }

        [Theory]
        [InlineData("g1_nowind", 0.005, 0.2, 0.2)]
        [InlineData("g1_nowind_up", 0.005, 0.4, 0.4)]
        [InlineData("g1_twist", 0.005, 0.2, 0.02)]
        [InlineData("g7_nowind", 0.005, 0.2, 0.2)]
        [InlineData("g1_wind", 0.005, 0.2, 0.2)]
        [InlineData("g1_wind_hot", 0.005, 0.2, 0.2)]
        [InlineData("g1_wind_cold", 0.005, 0.2, 0.2)]
        [InlineData("g1_nowind_coriolis", 0.005, 0.2, 0.2)]
        [InlineData("g1_wind_coriolis", 0.005, 0.2, 0.2)]
        public void TrajectoryTest(string name, double velocityAccuracyInPercent, double dropAccuracyInMOA, double windageAccuracyInMOA)
        {
            TableLoader template = TableLoader.FromResource(name);

            var cal = new TrajectoryCalculator();

            ShotParameters shot = new ShotParameters()
            {
                Step = new Measurement<DistanceUnit>(50, DistanceUnit.Yard),
                MaximumDistance = new Measurement<DistanceUnit>(1000, DistanceUnit.Yard),
                SightAngle = cal.SightAngle(template.Ammunition, template.Rifle, template.Atmosphere),
                ShotAngle = template.ShotParameters?.ShotAngle,
                CantAngle = template.ShotParameters?.CantAngle,
                Latitude = template.ShotParameters?.Latitude, // Use latitude from test data if present (for Coriolis effect)
                BarrelAzimuth = template.ShotParameters?.BarrelAzimuth // Use azimuth from test data if present
            };

            var winds = template.Wind == null ? null : new Wind[] { template.Wind };

            var trajectory = cal.Calculate(template.Ammunition, template.Rifle, template.Atmosphere, shot, winds);

            trajectory.Length.Should().Be(template.Trajectory.Count);

            for (int i = 0; i < trajectory.Length; i++)
            {
                var point = trajectory[i];
                var templatePoint = template.Trajectory[i];

                point.Distance.In(templatePoint.Distance.Unit).Should().BeApproximately(templatePoint.Distance.Value, templatePoint.Distance.Value * velocityAccuracyInPercent, $"@{point.Distance:N0}");
                point.Velocity.In(templatePoint.Velocity.Unit).Should().BeApproximately(templatePoint.Velocity.Value, templatePoint.Velocity.Value * velocityAccuracyInPercent, $"@{point.Distance:N0}");

                var dropAccuracyInInch = Measurement<AngularUnit>.Convert(dropAccuracyInMOA, AngularUnit.MOA, AngularUnit.InchesPer100Yards) * templatePoint.Distance.In(DistanceUnit.Yard) / 100;
                var windageAccuracyInInch = Measurement<AngularUnit>.Convert(windageAccuracyInMOA, AngularUnit.MOA, AngularUnit.InchesPer100Yards) * templatePoint.Distance.In(DistanceUnit.Yard) / 100;
                if (dropAccuracyInInch < 0.001)
                    dropAccuracyInInch = 0.001;
                if (windageAccuracyInInch < 0.001)
                    windageAccuracyInInch = 0.001;
                point.Drop.In(DistanceUnit.Inch).Should().BeApproximately(templatePoint.Drop.In(DistanceUnit.Inch), dropAccuracyInInch, $"@{point.Distance:N0}");
                point.Windage.In(DistanceUnit.Inch).Should().BeApproximately(templatePoint.Windage.In(DistanceUnit.Inch), windageAccuracyInInch, $"@{point.Distance:N0}");
            }
        }

        [Fact]
        public void CustomTable()
        {
            TableLoader template = TableLoader.FromResource("g1_nowind");
            const double velocityAccuracyInPercent = 0.005, dropAccuracyInMOA = 0.2, windageAccuracyInMOA = 0.2;

            var table = DragTable.Get(template.Ammunition.BallisticCoefficient.Table);
            template.Ammunition.BallisticCoefficient = new BallisticCoefficient(template.Ammunition.BallisticCoefficient.Value, DragTableId.GC);


            var cal = new TrajectoryCalculator();

            ShotParameters shot = new ShotParameters()
            {
                Step = new Measurement<DistanceUnit>(50, DistanceUnit.Yard),
                MaximumDistance = new Measurement<DistanceUnit>(1000, DistanceUnit.Yard),
                SightAngle = cal.SightAngle(template.Ammunition, template.Rifle, template.Atmosphere, table),
                ShotAngle = template.ShotParameters?.ShotAngle,
                CantAngle = template.ShotParameters?.CantAngle,
            };

            var trajectory = cal.Calculate(template.Ammunition, template.Rifle, template.Atmosphere, shot, null, table);

            trajectory.Length.Should().Be(template.Trajectory.Count);

            for (int i = 0; i < trajectory.Length; i++)
            {
                var point = trajectory[i];
                var templatePoint = template.Trajectory[i];

                point.Distance.In(templatePoint.Distance.Unit).Should().BeApproximately(templatePoint.Distance.Value, templatePoint.Distance.Value * velocityAccuracyInPercent, $"@{point.Distance:N0}");
                point.Velocity.In(templatePoint.Velocity.Unit).Should().BeApproximately(templatePoint.Velocity.Value, templatePoint.Velocity.Value * velocityAccuracyInPercent, $"@{point.Distance:N0}");

                var dropAccuracyInInch = Measurement<AngularUnit>.Convert(dropAccuracyInMOA, AngularUnit.MOA, AngularUnit.InchesPer100Yards) * templatePoint.Distance.In(DistanceUnit.Yard) / 100;
                var windageAccuracyInInch = Measurement<AngularUnit>.Convert(windageAccuracyInMOA, AngularUnit.MOA, AngularUnit.InchesPer100Yards) * templatePoint.Distance.In(DistanceUnit.Yard) / 100;

                point.Drop.In(DistanceUnit.Inch).Should().BeApproximately(templatePoint.Drop.In(DistanceUnit.Inch), dropAccuracyInInch, $"@{point.Distance:N0}");
                point.Windage.In(DistanceUnit.Inch).Should().BeApproximately(templatePoint.Windage.In(DistanceUnit.Inch), windageAccuracyInInch, $"@{point.Distance:N0}");
            }
        }

        [Fact]
        public void CustomTableNotSpecifiedException()
        {
            TableLoader template = TableLoader.FromResource("g1_nowind");
            template.Ammunition.BallisticCoefficient = new BallisticCoefficient(template.Ammunition.BallisticCoefficient.Value, DragTableId.GC);

            var cal = new TrajectoryCalculator();

            ((Action)(() => cal.SightAngle(template.Ammunition, template.Rifle, template.Atmosphere)))
                .Should().Throw<ArgumentNullException>();

            ShotParameters shot = new ShotParameters()
            {
                Step = new Measurement<DistanceUnit>(50, DistanceUnit.Yard),
                MaximumDistance = new Measurement<DistanceUnit>(1000, DistanceUnit.Yard),
                SightAngle = new Measurement<AngularUnit>(10, AngularUnit.MOA),
                ShotAngle = template.ShotParameters?.ShotAngle,
                CantAngle = template.ShotParameters?.CantAngle,
            };

            ((Action)(() => cal.Calculate(template.Ammunition, template.Rifle, template.Atmosphere, shot, null)))
                .Should().Throw<ArgumentNullException>();

        }

        class MyDrag : DragTable
        {
            public override DragTableId TableId => DragTableId.GC;

            private static readonly DragTableDataPoint[] gDataPoints = new DragTableDataPoint[]
            {
                new DragTableDataPoint(0.000, 0.180),
                new DragTableDataPoint(0.400, 0.178),
                new DragTableDataPoint(0.500, 0.154),
                new DragTableDataPoint(0.600, 0.129),
                new DragTableDataPoint(0.700, 0.131),
                new DragTableDataPoint(0.800, 0.136),
                new DragTableDataPoint(0.825, 0.140),
                new DragTableDataPoint(0.850, 0.144),
                new DragTableDataPoint(0.875, 0.153),
                new DragTableDataPoint(0.900, 0.177),
                new DragTableDataPoint(0.925, 0.226),
                new DragTableDataPoint(0.950, 0.260),
                new DragTableDataPoint(0.975, 0.349),
                new DragTableDataPoint(1.000, 0.427),
                new DragTableDataPoint(1.025, 0.450),
                new DragTableDataPoint(1.050, 0.452),
                new DragTableDataPoint(1.075, 0.450),
                new DragTableDataPoint(1.100, 0.447),
                new DragTableDataPoint(1.150, 0.437),
                new DragTableDataPoint(1.200, 0.429),
                new DragTableDataPoint(1.300, 0.418),
                new DragTableDataPoint(1.400, 0.406),
                new DragTableDataPoint(1.500, 0.394),
                new DragTableDataPoint(1.600, 0.382),
                new DragTableDataPoint(1.800, 0.359),
                new DragTableDataPoint(2.000, 0.339),
                new DragTableDataPoint(2.200, 0.321),
                new DragTableDataPoint(2.400, 0.301),
                new DragTableDataPoint(2.600, 0.280),
                new DragTableDataPoint(3.000, 0.250),
                new DragTableDataPoint(4.000, 0.200),
                new DragTableDataPoint(5.000, 0.180),
            };

            public MyDrag() : base(gDataPoints)
            {
            }
        }

        [Fact]
        public void Custom1()
        {
            var template = TableLoader.FromResource("custom");
            var table = new MyDrag();
            const double velocityAccuracyInPercent = 0.015, dropAccuracyInMOA = 0.25;

            var cal = new TrajectoryCalculator();

            ShotParameters shot = new ShotParameters()
            {
                Step = new Measurement<DistanceUnit>(50, DistanceUnit.Meter),
                MaximumDistance = new Measurement<DistanceUnit>(500, DistanceUnit.Meter),
                SightAngle = cal.SightAngle(template.Ammunition, template.Rifle, template.Atmosphere, table),
                ShotAngle = template.ShotParameters?.ShotAngle,
                CantAngle = template.ShotParameters?.CantAngle,
            };

            var winds = template.Wind == null ? null : new Wind[] { template.Wind };

            var trajectory = cal.Calculate(template.Ammunition, template.Rifle, template.Atmosphere, shot, winds, table);

            trajectory.Length.Should().Be(template.Trajectory.Count);

            for (int i = 0; i < trajectory.Length; i++)
            {
                var point = trajectory[i];
                var templatePoint = template.Trajectory[i];

                point.Distance.In(templatePoint.Distance.Unit).Should().BeApproximately(templatePoint.Distance.Value, templatePoint.Distance.Value * velocityAccuracyInPercent, $"@{point.Distance:N0}");
                point.Velocity.In(templatePoint.Velocity.Unit).Should().BeApproximately(templatePoint.Velocity.Value, templatePoint.Velocity.Value * velocityAccuracyInPercent, $"@{point.Distance:N0}");
                if (i > 0)
                {
                    var dropAccuracyInInch = Measurement<AngularUnit>.Convert(dropAccuracyInMOA, AngularUnit.MOA, AngularUnit.InchesPer100Yards) * templatePoint.Distance.In(DistanceUnit.Yard) / 100;
                    point.Drop.In(DistanceUnit.Inch).Should().BeApproximately(templatePoint.Drop.In(DistanceUnit.Inch), dropAccuracyInInch, $"@{point.Distance:N0}");
                }
            }
        }

        [Fact]
        public void Custom2()
        {
            var template = TableLoader.FromResource("custom2");
            using var stream = typeof(TrajectoryCalculatorTest).Assembly.GetManifestResourceStream($"BallisticCalculator.Test.resources.drg2.txt");
            var table = DrgDragTable.Open(stream);
            
            const double velocityAccuracyInPercent = 0.015, dropAccuracyInMOA = 0.25;

            var cal = new TrajectoryCalculator();

            ShotParameters shot = new ShotParameters()
            {
                Step = new Measurement<DistanceUnit>(100, DistanceUnit.Meter),
                MaximumDistance = new Measurement<DistanceUnit>(1500, DistanceUnit.Meter),
                SightAngle = cal.SightAngle(template.Ammunition, template.Rifle, template.Atmosphere, table),
                ShotAngle = template.ShotParameters?.ShotAngle,
                CantAngle = template.ShotParameters?.CantAngle,
            };

            var winds = template.Wind == null ? null : new Wind[] { template.Wind };

            var trajectory = cal.Calculate(template.Ammunition, template.Rifle, template.Atmosphere, shot, winds, table);

            trajectory.Length.Should().Be(template.Trajectory.Count);

            for (int i = 0; i < trajectory.Length; i++)
            {
                var point = trajectory[i];
                var templatePoint = template.Trajectory[i];

                point.Distance.In(templatePoint.Distance.Unit).Should().BeApproximately(templatePoint.Distance.Value, templatePoint.Distance.Value * velocityAccuracyInPercent, $"@{point.Distance:N0}");
                point.Velocity.In(templatePoint.Velocity.Unit).Should().BeApproximately(templatePoint.Velocity.Value, templatePoint.Velocity.Value * velocityAccuracyInPercent, $"@{point.Distance:N0}");
                if (i > 0)
                {
                    var dropAccuracyInInch = Measurement<AngularUnit>.Convert(dropAccuracyInMOA, AngularUnit.MOA, AngularUnit.InchesPer100Yards) * templatePoint.Distance.In(DistanceUnit.Yard) / 100;
                    point.Drop.In(DistanceUnit.Inch).Should().BeApproximately(templatePoint.Drop.In(DistanceUnit.Inch), dropAccuracyInInch, $"@{point.Distance:N0}");
                }
            }
        }

        [Fact]
        public void CoriolisEffect_NoLatitude_NoEffect()
        {
            var template = TableLoader.FromResource("g1_nowind");
            var cal = new TrajectoryCalculator();

            var shotWithoutLatitude = new ShotParameters()
            {
                Step = new Measurement<DistanceUnit>(50, DistanceUnit.Yard),
                MaximumDistance = new Measurement<DistanceUnit>(1000, DistanceUnit.Yard),
                SightAngle = cal.SightAngle(template.Ammunition, template.Rifle, template.Atmosphere),
                ShotAngle = template.ShotParameters?.ShotAngle,
                CantAngle = template.ShotParameters?.CantAngle,
                BarrelAzimuth = template.ShotParameters?.BarrelAzimuth,
                Latitude = null
            };

            var shotWithLatitude = new ShotParameters()
            {
                Step = new Measurement<DistanceUnit>(50, DistanceUnit.Yard),
                MaximumDistance = new Measurement<DistanceUnit>(1000, DistanceUnit.Yard),
                SightAngle = cal.SightAngle(template.Ammunition, template.Rifle, template.Atmosphere),
                ShotAngle = template.ShotParameters?.ShotAngle,
                CantAngle = template.ShotParameters?.CantAngle,
                BarrelAzimuth = template.ShotParameters?.BarrelAzimuth,
                Latitude = new Measurement<AngularUnit>(45, AngularUnit.Degree)
            };

            var winds = template.Wind == null ? null : new Wind[] { template.Wind };
            var trajectoryWithoutLatitude = cal.Calculate(template.Ammunition, template.Rifle, template.Atmosphere, shotWithoutLatitude, winds);
            var trajectoryWithLatitude = cal.Calculate(template.Ammunition, template.Rifle, template.Atmosphere, shotWithLatitude, winds);

            trajectoryWithoutLatitude[0].Windage.In(DistanceUnit.Inch).Should().BeApproximately(
                trajectoryWithLatitude[0].Windage.In(DistanceUnit.Inch), 0.001);
        }

        [Fact]
        public void CoriolisEffect_LatitudeAffectsWindage()
        {
            var template = TableLoader.FromResource("g1_nowind");
            var cal = new TrajectoryCalculator();

            var shotWithoutLatitude = new ShotParameters()
            {
                Step = new Measurement<DistanceUnit>(50, DistanceUnit.Yard),
                MaximumDistance = new Measurement<DistanceUnit>(1000, DistanceUnit.Yard),
                SightAngle = cal.SightAngle(template.Ammunition, template.Rifle, template.Atmosphere),
                ShotAngle = template.ShotParameters?.ShotAngle,
                CantAngle = template.ShotParameters?.CantAngle,
                BarrelAzimuth = template.ShotParameters?.BarrelAzimuth,
                Latitude = null
            };

            var shotWithLatitude = new ShotParameters()
            {
                Step = new Measurement<DistanceUnit>(50, DistanceUnit.Yard),
                MaximumDistance = new Measurement<DistanceUnit>(1000, DistanceUnit.Yard),
                SightAngle = cal.SightAngle(template.Ammunition, template.Rifle, template.Atmosphere),
                ShotAngle = template.ShotParameters?.ShotAngle,
                CantAngle = template.ShotParameters?.CantAngle,
                BarrelAzimuth = template.ShotParameters?.BarrelAzimuth,
                Latitude = new Measurement<AngularUnit>(45, AngularUnit.Degree)
            };

            var winds = template.Wind == null ? null : new Wind[] { template.Wind };
            var trajectoryWithoutLatitude = cal.Calculate(template.Ammunition, template.Rifle, template.Atmosphere, shotWithoutLatitude, winds);
            var trajectoryWithLatitude = cal.Calculate(template.Ammunition, template.Rifle, template.Atmosphere, shotWithLatitude, winds);

            var lastWithout = trajectoryWithoutLatitude.Last();
            var lastWith = trajectoryWithLatitude.Last();

            Math.Abs(lastWith.Windage.In(DistanceUnit.Inch) - lastWithout.Windage.In(DistanceUnit.Inch))
                .Should().BeGreaterThan(0.005);
        }

        [Fact]
        public void CoriolisEffect_LatitudeAffectsDrop()
        {
            var template = TableLoader.FromResource("g1_nowind");
            var cal = new TrajectoryCalculator();

            var shotWithoutLatitude = new ShotParameters()
            {
                Step = new Measurement<DistanceUnit>(50, DistanceUnit.Yard),
                MaximumDistance = new Measurement<DistanceUnit>(1000, DistanceUnit.Yard),
                SightAngle = cal.SightAngle(template.Ammunition, template.Rifle, template.Atmosphere),
                ShotAngle = template.ShotParameters?.ShotAngle,
                CantAngle = template.ShotParameters?.CantAngle,
                BarrelAzimuth = new Measurement<AngularUnit>(45, AngularUnit.Degree),
                Latitude = null
            };

            var shotWithLatitude = new ShotParameters()
            {
                Step = new Measurement<DistanceUnit>(50, DistanceUnit.Yard),
                MaximumDistance = new Measurement<DistanceUnit>(1000, DistanceUnit.Yard),
                SightAngle = cal.SightAngle(template.Ammunition, template.Rifle, template.Atmosphere),
                ShotAngle = template.ShotParameters?.ShotAngle,
                CantAngle = template.ShotParameters?.CantAngle,
                BarrelAzimuth = new Measurement<AngularUnit>(45, AngularUnit.Degree),
                Latitude = new Measurement<AngularUnit>(45, AngularUnit.Degree)
            };

            var winds = template.Wind == null ? null : new Wind[] { template.Wind };
            var trajectoryWithoutLatitude = cal.Calculate(template.Ammunition, template.Rifle, template.Atmosphere, shotWithoutLatitude, winds);
            var trajectoryWithLatitude = cal.Calculate(template.Ammunition, template.Rifle, template.Atmosphere, shotWithLatitude, winds);

            var lastWithout = trajectoryWithoutLatitude.Last();
            var lastWith = trajectoryWithLatitude.Last();

            Math.Abs(lastWith.Drop.In(DistanceUnit.Inch) - lastWithout.Drop.In(DistanceUnit.Inch))
                .Should().BeGreaterThan(0.005);
        }

        [Fact]
        public void CoriolisEffect_DependsOnLatitude()
        {
            var template = TableLoader.FromResource("g1_nowind");
            var cal = new TrajectoryCalculator();

            var shotLowLatitude = new ShotParameters()
            {
                Step = new Measurement<DistanceUnit>(50, DistanceUnit.Yard),
                MaximumDistance = new Measurement<DistanceUnit>(1000, DistanceUnit.Yard),
                SightAngle = cal.SightAngle(template.Ammunition, template.Rifle, template.Atmosphere),
                ShotAngle = template.ShotParameters?.ShotAngle,
                CantAngle = template.ShotParameters?.CantAngle,
                BarrelAzimuth = new Measurement<AngularUnit>(0, AngularUnit.Degree),
                Latitude = new Measurement<AngularUnit>(10, AngularUnit.Degree)
            };

            var shotHighLatitude = new ShotParameters()
            {
                Step = new Measurement<DistanceUnit>(50, DistanceUnit.Yard),
                MaximumDistance = new Measurement<DistanceUnit>(1000, DistanceUnit.Yard),
                SightAngle = cal.SightAngle(template.Ammunition, template.Rifle, template.Atmosphere),
                ShotAngle = template.ShotParameters?.ShotAngle,
                CantAngle = template.ShotParameters?.CantAngle,
                BarrelAzimuth = new Measurement<AngularUnit>(0, AngularUnit.Degree),
                Latitude = new Measurement<AngularUnit>(60, AngularUnit.Degree)
            };

            var winds = template.Wind == null ? null : new Wind[] { template.Wind };
            var trajectoryLowLatitude = cal.Calculate(template.Ammunition, template.Rifle, template.Atmosphere, shotLowLatitude, winds);
            var trajectoryHighLatitude = cal.Calculate(template.Ammunition, template.Rifle, template.Atmosphere, shotHighLatitude, winds);

            var lastLow = trajectoryLowLatitude.Last();
            var lastHigh = trajectoryHighLatitude.Last();

            Math.Abs(lastHigh.Windage.In(DistanceUnit.Inch))
                .Should().BeGreaterThan(Math.Abs(lastLow.Windage.In(DistanceUnit.Inch)));
        }

        [Fact]
        public void CoriolisEffect_DependsOnAzimuth()
        {
            var template = TableLoader.FromResource("g1_nowind");
            var cal = new TrajectoryCalculator();

            var shotNorth = new ShotParameters()
            {
                Step = new Measurement<DistanceUnit>(50, DistanceUnit.Yard),
                MaximumDistance = new Measurement<DistanceUnit>(1000, DistanceUnit.Yard),
                SightAngle = cal.SightAngle(template.Ammunition, template.Rifle, template.Atmosphere),
                ShotAngle = template.ShotParameters?.ShotAngle,
                CantAngle = template.ShotParameters?.CantAngle,
                BarrelAzimuth = new Measurement<AngularUnit>(0, AngularUnit.Degree),
                Latitude = new Measurement<AngularUnit>(45, AngularUnit.Degree)
            };

            var shotNE = new ShotParameters()
            {
                Step = new Measurement<DistanceUnit>(50, DistanceUnit.Yard),
                MaximumDistance = new Measurement<DistanceUnit>(1000, DistanceUnit.Yard),
                SightAngle = cal.SightAngle(template.Ammunition, template.Rifle, template.Atmosphere),
                ShotAngle = template.ShotParameters?.ShotAngle,
                CantAngle = template.ShotParameters?.CantAngle,
                BarrelAzimuth = new Measurement<AngularUnit>(45, AngularUnit.Degree),
                Latitude = new Measurement<AngularUnit>(45, AngularUnit.Degree)
            };

            var winds = template.Wind == null ? null : new Wind[] { template.Wind };
            var trajectoryNorth = cal.Calculate(template.Ammunition, template.Rifle, template.Atmosphere, shotNorth, winds);
            var trajectoryNE = cal.Calculate(template.Ammunition, template.Rifle, template.Atmosphere, shotNE, winds);

            var lastNorth = trajectoryNorth.Last();
            var lastNE = trajectoryNE.Last();

            var windageDiff = Math.Abs(Math.Abs(lastNorth.Windage.In(DistanceUnit.Inch)) - Math.Abs(lastNE.Windage.In(DistanceUnit.Inch)));
            var dropDiff = Math.Abs(Math.Abs(lastNorth.Drop.In(DistanceUnit.Inch)) - Math.Abs(lastNE.Drop.In(DistanceUnit.Inch)));

            (windageDiff + dropDiff).Should().BeGreaterThan(0.01);
        }

        [Fact]
        public void AerodynamicJump_AffectsTrajectory_WhenRiflingDataPresent()
        {
            // Arrange - Test that aerodynamic jump affects trajectory when rifling data is present
            Ammunition ammunition = new Ammunition(
                weight: new Measurement<WeightUnit>(69, WeightUnit.Grain),
                muzzleVelocity: new Measurement<VelocityUnit>(2600, VelocityUnit.FeetPerSecond),
                ballisticCoefficient: new BallisticCoefficient(0.365, DragTableId.G1),
                bulletDiameter: new Measurement<DistanceUnit>(0.224, DistanceUnit.Inch),
                bulletLength: new Measurement<DistanceUnit>(0.8, DistanceUnit.Inch));

            Rifle rifleWithRifling = new Rifle(
                sight: new Sight(sightHeight: new Measurement<DistanceUnit>(3.2, DistanceUnit.Inch), Measurement<AngularUnit>.ZERO, Measurement<AngularUnit>.ZERO),
                zero: new ZeroingParameters(distance: new Measurement<DistanceUnit>(100, DistanceUnit.Yard), ammunition: null, atmosphere: null),
                rifling: new Rifling(new Measurement<DistanceUnit>(12, DistanceUnit.Inch), TwistDirection.Right));

            Rifle rifleWithoutRifling = new Rifle(
                sight: new Sight(sightHeight: new Measurement<DistanceUnit>(3.2, DistanceUnit.Inch), Measurement<AngularUnit>.ZERO, Measurement<AngularUnit>.ZERO),
                zero: new ZeroingParameters(distance: new Measurement<DistanceUnit>(100, DistanceUnit.Yard), ammunition: null, atmosphere: null),
                rifling: null); // No rifling - no aerodynamic jump

            Atmosphere atmosphere = new Atmosphere();
            var calculator = new TrajectoryCalculator();

            ShotParameters shot = new ShotParameters()
            {
                Step = new Measurement<DistanceUnit>(50, DistanceUnit.Yard),
                MaximumDistance = new Measurement<DistanceUnit>(500, DistanceUnit.Yard),
                SightAngle = calculator.SightAngle(ammunition, rifleWithRifling, atmosphere),
                ShotAngle = null,
                CantAngle = null,
            };

            // Act
            var trajectoryWithRifling = calculator.Calculate(ammunition, rifleWithRifling, atmosphere, shot, null);
            var trajectoryWithoutRifling = calculator.Calculate(ammunition, rifleWithoutRifling, atmosphere, shot, null);

            // Assert
            // Trajectories should differ due to aerodynamic jump
            // The difference should be small but measurable at longer distances
            trajectoryWithRifling.Length.Should().BeGreaterThan(0);
            trajectoryWithoutRifling.Length.Should().BeGreaterThan(0);
            
            // At longer distances, the trajectories should differ
            if (trajectoryWithRifling.Length > 5 && trajectoryWithoutRifling.Length > 5)
            {
                var pointWithRifling = trajectoryWithRifling[trajectoryWithRifling.Length - 1];
                var pointWithoutRifling = trajectoryWithoutRifling[trajectoryWithoutRifling.Length - 1];
                
                // Windage or drop should differ (aerodynamic jump affects initial velocity)
                var windageDiff = Math.Abs(pointWithRifling.Windage.In(DistanceUnit.Inch) - pointWithoutRifling.Windage.In(DistanceUnit.Inch));
                // At 500 yards, aerodynamic jump should produce measurable difference (even if small)
                // The exact value depends on the implementation, but there should be some difference
                (windageDiff > 0.001 || Math.Abs(pointWithRifling.Drop.In(DistanceUnit.Inch) - pointWithoutRifling.Drop.In(DistanceUnit.Inch)) > 0.001)
                    .Should().BeTrue("Trajectories with and without rifling should differ due to aerodynamic jump");
            }
        }

        [Fact]
        public void AerodynamicJump_DifferentTwistDirections_ProduceDifferentTrajectories()
        {
            // Arrange
            Ammunition ammunition = new Ammunition(
                weight: new Measurement<WeightUnit>(69, WeightUnit.Grain),
                muzzleVelocity: new Measurement<VelocityUnit>(2600, VelocityUnit.FeetPerSecond),
                ballisticCoefficient: new BallisticCoefficient(0.365, DragTableId.G1),
                bulletDiameter: new Measurement<DistanceUnit>(0.224, DistanceUnit.Inch),
                bulletLength: new Measurement<DistanceUnit>(0.8, DistanceUnit.Inch));

            Rifle rifleRight = new Rifle(
                sight: new Sight(sightHeight: new Measurement<DistanceUnit>(3.2, DistanceUnit.Inch), Measurement<AngularUnit>.ZERO, Measurement<AngularUnit>.ZERO),
                zero: new ZeroingParameters(distance: new Measurement<DistanceUnit>(100, DistanceUnit.Yard), ammunition: null, atmosphere: null),
                rifling: new Rifling(new Measurement<DistanceUnit>(12, DistanceUnit.Inch), TwistDirection.Right));

            Rifle rifleLeft = new Rifle(
                sight: new Sight(sightHeight: new Measurement<DistanceUnit>(3.2, DistanceUnit.Inch), Measurement<AngularUnit>.ZERO, Measurement<AngularUnit>.ZERO),
                zero: new ZeroingParameters(distance: new Measurement<DistanceUnit>(100, DistanceUnit.Yard), ammunition: null, atmosphere: null),
                rifling: new Rifling(new Measurement<DistanceUnit>(12, DistanceUnit.Inch), TwistDirection.Left));

            Atmosphere atmosphere = new Atmosphere();
            var calculator = new TrajectoryCalculator();

            ShotParameters shot = new ShotParameters()
            {
                Step = new Measurement<DistanceUnit>(50, DistanceUnit.Yard),
                MaximumDistance = new Measurement<DistanceUnit>(500, DistanceUnit.Yard),
                SightAngle = calculator.SightAngle(ammunition, rifleRight, atmosphere),
                ShotAngle = null,
                CantAngle = null,
            };

            // Act
            var trajectoryRight = calculator.Calculate(ammunition, rifleRight, atmosphere, shot, null);
            var trajectoryLeft = calculator.Calculate(ammunition, rifleLeft, atmosphere, shot, null);

            // Assert
            // Different twist directions should produce different trajectories
            // The windage should be opposite (or at least different)
            trajectoryRight.Length.Should().BeGreaterThan(0);
            trajectoryLeft.Length.Should().BeGreaterThan(0);
            
            if (trajectoryRight.Length > 5 && trajectoryLeft.Length > 5)
            {
                var pointRight = trajectoryRight[trajectoryRight.Length - 1];
                var pointLeft = trajectoryLeft[trajectoryLeft.Length - 1];
                
                // Windage should differ (opposite directions for opposite twist)
                var windageDiff = Math.Abs(pointRight.Windage.In(DistanceUnit.Inch) - pointLeft.Windage.In(DistanceUnit.Inch));
                windageDiff.Should().BeGreaterThan(0.001, "Different twist directions should produce different windage");
            }
        }

        [Fact]
        public void AerodynamicJump_VariesWithAltitude()
        {
            // Arrange
            Ammunition ammunition = new Ammunition(
                weight: new Measurement<WeightUnit>(69, WeightUnit.Grain),
                muzzleVelocity: new Measurement<VelocityUnit>(2600, VelocityUnit.FeetPerSecond),
                ballisticCoefficient: new BallisticCoefficient(0.365, DragTableId.G1),
                bulletDiameter: new Measurement<DistanceUnit>(0.224, DistanceUnit.Inch),
                bulletLength: new Measurement<DistanceUnit>(0.8, DistanceUnit.Inch));

            Rifle rifle = new Rifle(
                sight: new Sight(sightHeight: new Measurement<DistanceUnit>(3.2, DistanceUnit.Inch), Measurement<AngularUnit>.ZERO, Measurement<AngularUnit>.ZERO),
                zero: new ZeroingParameters(distance: new Measurement<DistanceUnit>(100, DistanceUnit.Yard), ammunition: null, atmosphere: null),
                rifling: new Rifling(new Measurement<DistanceUnit>(12, DistanceUnit.Inch), TwistDirection.Right));

            Atmosphere atmosphereSeaLevel = new Atmosphere(
                altitude: new Measurement<DistanceUnit>(0, DistanceUnit.Foot),
                pressure: new Measurement<PressureUnit>(29.92, PressureUnit.InchesOfMercury),
                pressureAtSeaLevel: false,
                temperature: new Measurement<TemperatureUnit>(59, TemperatureUnit.Fahrenheit),
                humidity: 0.0);

            Atmosphere atmosphereHighAltitude = new Atmosphere(
                altitude: new Measurement<DistanceUnit>(10000, DistanceUnit.Foot),
                pressure: new Measurement<PressureUnit>(20.58, PressureUnit.InchesOfMercury), // Approximate at 10k ft
                pressureAtSeaLevel: false,
                temperature: new Measurement<TemperatureUnit>(23.3, TemperatureUnit.Fahrenheit), // Approximate at 10k ft
                humidity: 0.0);

            var calculator = new TrajectoryCalculator();

            ShotParameters shot = new ShotParameters()
            {
                Step = new Measurement<DistanceUnit>(50, DistanceUnit.Yard),
                MaximumDistance = new Measurement<DistanceUnit>(500, DistanceUnit.Yard),
                SightAngle = calculator.SightAngle(ammunition, rifle, atmosphereSeaLevel),
                ShotAngle = null,
                CantAngle = null,
            };

            // Act
            var trajectorySeaLevel = calculator.Calculate(ammunition, rifle, atmosphereSeaLevel, shot, null);
            var trajectoryHighAltitude = calculator.Calculate(ammunition, rifle, atmosphereHighAltitude, shot, null);

            // Assert
            // Higher altitude has lower air density, so aerodynamic jump should be smaller
            // This should result in slightly different trajectories
            trajectorySeaLevel.Length.Should().BeGreaterThan(0);
            trajectoryHighAltitude.Length.Should().BeGreaterThan(0);
            
            // The trajectories should differ due to different air density affecting aerodynamic jump
            // The exact difference depends on implementation, but there should be some measurable difference
            if (trajectorySeaLevel.Length > 5 && trajectoryHighAltitude.Length > 5)
            {
                var pointSeaLevel = trajectorySeaLevel[trajectorySeaLevel.Length - 1];
                var pointHighAltitude = trajectoryHighAltitude[trajectoryHighAltitude.Length - 1];
                
                // At least one component should differ
                var windageDiff = Math.Abs(pointSeaLevel.Windage.In(DistanceUnit.Inch) - pointHighAltitude.Windage.In(DistanceUnit.Inch));
                var dropDiff = Math.Abs(pointSeaLevel.Drop.In(DistanceUnit.Inch) - pointHighAltitude.Drop.In(DistanceUnit.Inch));
                
                (windageDiff > 0.001 || dropDiff > 0.001)
                    .Should().BeTrue("Trajectories at different altitudes should differ due to aerodynamic jump variation");
            }
        }
    }
}
