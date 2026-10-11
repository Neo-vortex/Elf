using System.Numerics;
using NeoRadio.Dsp;
using Xunit;

namespace NeoRadio.Dsp.Tests;

public class FilterTests
{
    internal static Complex[] Tone(double f, double fs, int n, double amp = 1.0) =>
        Enumerable.Range(0, n).Select(i => Complex.FromPolarCoordinates(amp, 2 * Math.PI * f * i / fs)).ToArray();

    static double TailRms(Complex[] x, int skip) =>
        Math.Sqrt(x.Skip(skip).Average(c => c.Magnitude * c.Magnitude));

    [Fact]
    public void ComplexFir_passes_inband_and_blocks_outband()
    {
        var taps = FirDesign.Lowpass(1000, 200, 12000);
        var pass = Tone(500, 12000, 4000);
        new ComplexFir(taps).Process(pass, pass);
        var stop = Tone(-2000, 12000, 4000);
        new ComplexFir(taps).Process(stop, stop);
        Assert.InRange(TailRms(pass, taps.Length), 0.99, 1.01);
        Assert.True(TailRms(stop, taps.Length) < 1e-3);
    }

    [Fact]
    public void ComplexFir_streaming_equals_one_shot()
    {
        var taps = FirDesign.Lowpass(1000, 200, 12000);
        var x = Tone(700, 12000, 3000);
        var once = new Complex[x.Length];
        new ComplexFir(taps).Process(x, once);
        var parts = new Complex[x.Length];
        var fir = new ComplexFir(taps);
        fir.Process(x.AsSpan(0, 1111), parts.AsSpan(0, 1111));
        fir.Process(x.AsSpan(1111), parts.AsSpan(1111));
        for (int i = 0; i < x.Length; i++) Assert.True((once[i] - parts[i]).Magnitude < 1e-12);
    }

    [Fact]
    public void Decimator_outputs_one_sample_per_factor_and_keeps_inband_tone()
    {
        var taps = FirDesign.Lowpass(4800, 2400, 96000);
        var dec = new Decimator(8, taps);
        var x = Tone(1000, 96000, 96000);
        var y1 = dec.Process(x.AsSpan(0, 50001));
        var y2 = dec.Process(x.AsSpan(50001));
        Assert.Equal(12000, y1.Length + y2.Length);
        var y = y1.Concat(y2).ToArray();
        Assert.InRange(TailRms(y, taps.Length / 8 + 1), 0.99, 1.01);
        var step = y[5000] * Complex.Conjugate(y[4999]);
        Assert.Equal(2 * Math.PI * 1000 / 12000, step.Phase, 3);
    }

    [Fact]
    public void Decimator_rejects_band_that_would_alias()
    {
        var taps = FirDesign.Lowpass(4800, 2400, 96000);
        var y = new Decimator(8, taps).Process(Tone(13000, 96000, 48000));
        Assert.True(TailRms(y, taps.Length / 8 + 1) < 1e-3);
    }

    [Fact]
    public void Decimator_rejects_factor_below_one() =>
        Assert.Throws<ArgumentOutOfRangeException>(() => new Decimator(0, new double[] { 1 }));
}
