using System.Numerics;
using NeoRadio.Dsp;
using Xunit;

namespace NeoRadio.Dsp.Tests;

public class IqCorrectorTests
{
    const double Fs = 96000;

    /// <summary>The mixer's imbalance as neohw measures it and the twin applies it:
    /// q = −Im(z·e^{jφ}) / g with z = I − jQ (gain I over Q, phase of Q from quadrature).</summary>
    internal static Complex[] Imbalance(Complex[] z, double gainDb, double phaseDeg)
    {
        double g = Math.Pow(10, gainDb / 20);
        var rot = Complex.FromPolarCoordinates(1, phaseDeg * Math.PI / 180);
        return z.Select(s => new Complex(s.Real, (s * rot).Imaginary / g)).ToArray();
    }

    static double Bin(Complex[] z, double f)
    {
        Complex acc = Complex.Zero;
        for (int i = 0; i < z.Length; i++) acc += z[i] * Complex.FromPolarCoordinates(1, -2 * Math.PI * f * i / Fs);
        return acc.Magnitude / z.Length;
    }

    static double ImageRejectionDb(Complex[] z, double f) => 20 * Math.Log10(Bin(z, f) / Bin(z, -f));

    [Theory]
    [InlineData(0.16, 0.43)]     // mixer.json p5 run, direct path
    [InlineData(-0.5, 3.0)]
    [InlineData(1.0, -5.0)]
    public void Known_imbalance_is_corrected_to_at_least_60_dB_image_rejection(double gainDb, double phaseDeg)
    {
        var z = Imbalance(FilterTests.Tone(12000, Fs, 9600, 0.5), gainDb, phaseDeg);
        Assert.True(ImageRejectionDb(z, 12000) < 50);
        new IqCorrector(gainDb, phaseDeg).Correct(z);
        Assert.True(ImageRejectionDb(z, 12000) >= 60, $"{ImageRejectionDb(z, 12000):F1} dB");
        Assert.InRange(Bin(z, 12000), 0.49, 0.51);
    }

    [Fact]
    public void Zero_imbalance_leaves_samples_unchanged()
    {
        var z = FilterTests.Tone(-7000, Fs, 960, 0.3);
        var copy = z.ToArray();
        var c = new IqCorrector(0, 0);
        Assert.True(c.IsIdentity);
        c.Correct(z);
        Assert.Equal(copy, z);
    }

    [Theory]
    [InlineData(double.NaN, 0)]
    [InlineData(0, double.PositiveInfinity)]
    [InlineData(6.5, 0)]
    [InlineData(0, -31)]
    public void Out_of_range_or_non_finite_imbalance_is_rejected(double gainDb, double phaseDeg)
    {
        Assert.Throws<ArgumentException>(() => new IqCorrector(gainDb, phaseDeg));
    }
}
