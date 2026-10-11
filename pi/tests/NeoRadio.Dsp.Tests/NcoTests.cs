using System.Numerics;
using NeoRadio.Dsp;
using Xunit;

namespace NeoRadio.Dsp.Tests;

public class NcoTests
{
    [Fact]
    public void Mix_rotates_by_requested_frequency()
    {
        var data = Enumerable.Repeat(Complex.One, 9600).ToArray();
        new Nco(1000, 96000).Mix(data);
        for (int n = 0; n < data.Length; n += 97)
        {
            var expected = Complex.FromPolarCoordinates(1, 2 * Math.PI * 1000 * n / 96000);
            Assert.True((data[n] - expected).Magnitude < 1e-9, $"sample {n}");
        }
    }

    [Fact]
    public void Mix_is_phase_continuous_across_calls()
    {
        var whole = Enumerable.Repeat(Complex.One, 5000).ToArray();
        new Nco(-1234.5, 96000).Mix(whole);

        var split = Enumerable.Repeat(Complex.One, 5000).ToArray();
        var nco = new Nco(-1234.5, 96000);
        nco.Mix(split.AsSpan(0, 1700));
        nco.Mix(split.AsSpan(1700));

        for (int i = 0; i < whole.Length; i++)
            Assert.True((whole[i] - split[i]).Magnitude < 1e-9, $"sample {i}");
    }
}
