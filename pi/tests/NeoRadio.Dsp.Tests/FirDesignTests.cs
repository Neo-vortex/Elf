using System.Numerics;
using NeoRadio.Dsp;
using Xunit;

namespace NeoRadio.Dsp.Tests;

public class FirDesignTests
{
    internal static double Gain(double[] taps, double f, double fs)
    {
        Complex acc = Complex.Zero;
        for (int i = 0; i < taps.Length; i++)
            acc += taps[i] * Complex.FromPolarCoordinates(1, -2 * Math.PI * f * i / fs);
        return acc.Magnitude;
    }

    [Fact]
    public void Lowpass_has_unity_dc_gain() =>
        Assert.Equal(1.0, Gain(FirDesign.Lowpass(1000, 200, 12000), 0, 12000), 6);

    [Fact]
    public void Lowpass_passes_passband_and_rejects_stopband()
    {
        var taps = FirDesign.Lowpass(1000, 200, 12000);
        Assert.True(Gain(taps, 800, 12000) > 0.999);
        Assert.True(Gain(taps, 1200, 12000) < 1e-3);
        Assert.True(Gain(taps, 3000, 12000) < 1e-3);
    }

    [Fact]
    public void Lowpass_tap_count_is_odd_and_sized_by_transition()
    {
        var taps = FirDesign.Lowpass(1000, 200, 12000);
        Assert.Equal(1, taps.Length % 2);
        Assert.InRange(taps.Length, 330, 332);
    }

    [Theory]
    [InlineData(0, 100)]
    [InlineData(6000, 100)]
    [InlineData(1000, 0)]
    public void Lowpass_rejects_invalid_arguments(double cutoff, double transition) =>
        Assert.Throws<ArgumentOutOfRangeException>(() => FirDesign.Lowpass(cutoff, transition, 12000));
}
