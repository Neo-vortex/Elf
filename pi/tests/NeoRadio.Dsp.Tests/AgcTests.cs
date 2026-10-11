using NeoRadio.Dsp;
using Xunit;

namespace NeoRadio.Dsp.Tests;

public class AgcTests
{
    static float[] Sine(double amp, double f, double fs, int n) =>
        Enumerable.Range(0, n).Select(i => (float)(amp * Math.Sin(2 * Math.PI * f * i / fs))).ToArray();

    [Theory]
    [InlineData(0.001)]
    [InlineData(1.0)]
    public void Settles_to_target_peak(double amplitude)
    {
        var x = Sine(amplitude, 600, 12000, 24000);
        new Agc(12000).Process(x);
        float peak = x.Skip(12000).Max(v => Math.Abs(v));
        Assert.InRange(peak, 0.29f, 0.3001f);
    }

    [Fact]
    public void Output_never_exceeds_target_even_at_start()
    {
        var x = Sine(0.5, 600, 12000, 100);
        new Agc(12000).Process(x);
        Assert.All(x, v => Assert.InRange(v, -0.3001f, 0.3001f));
    }
}
