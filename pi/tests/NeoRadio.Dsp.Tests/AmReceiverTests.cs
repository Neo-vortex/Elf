using System.Numerics;
using NeoRadio.Dsp;
using Xunit;

namespace NeoRadio.Dsp.Tests;

public class AmReceiverTests
{
    const double Fs = 96000;
    const double Carrier = 10000;           // carrier offset from the LO, as the board tunes

    /// <summary>An AM station: carrier at <paramref name="offset"/> with modulation depth m by a tone;
    /// <paramref name="carrierGain"/> scales the carrier alone (selective fading of the carrier).</summary>
    static Complex[] Am(double offset, double tone, double m, int n, double carrierGain = 1.0, double amp = 0.1)
    {
        var x = new Complex[n];
        for (int i = 0; i < n; i++)
        {
            double t = i / Fs;
            double env = carrierGain + m * Math.Cos(2 * Math.PI * tone * t);
            x[i] = amp * env * Complex.FromPolarCoordinates(1, 2 * Math.PI * offset * t);
        }
        return x;
    }

    static double ToneAmplitude(float[] audio, double f, double fs, int skip)
    {
        Complex acc = Complex.Zero;
        int n = 0;
        for (int i = skip; i < audio.Length; i++, n++)
            acc += audio[i] * Complex.FromPolarCoordinates(1, -2 * Math.PI * f * i / fs);
        return 2 * acc.Magnitude / n;
    }

    static AmConfig Config(bool sync = false, bool agc = false) => new(Fs, 12000, Carrier, 9000, sync, agc);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Programme_tone_is_recovered(bool sync)
    {
        var audio = new AmReceiver(Config(sync)).Process(Am(Carrier, 1000, 0.5, 96000 * 2));
        Assert.Equal(24000, audio.Length);
        double wanted = ToneAmplitude(audio, 1000, 12000, 6000);
        Assert.InRange(wanted, 0.04, 0.06);                         // m · amplitude = 0.05
        Assert.True(wanted > 100 * ToneAmplitude(audio, 2000, 12000, 6000));   // < 1 % second harmonic
    }

    [Fact]
    public void Carrier_is_removed_as_dc()
    {
        var audio = new AmReceiver(Config()).Process(Am(Carrier, 1000, 0.3, 96000 * 2));
        double mean = audio.Skip(6000).Average(v => (double)v);
        Assert.InRange(mean, -0.002, 0.002);
    }

    [Fact]
    public void Adjacent_channel_station_is_rejected_by_40_dB()
    {
        var x = Am(Carrier, 1000, 0.5, 96000 * 2);
        var adj = Am(Carrier + 9000, 1500, 0.9, 96000 * 2);         // the next 9 kHz channel
        for (int i = 0; i < x.Length; i++) x[i] += adj[i];
        var audio = new AmReceiver(Config()).Process(x);
        double wanted = ToneAmplitude(audio, 1000, 12000, 6000);
        double leak = ToneAmplitude(audio, 1500, 12000, 6000);
        Assert.True(wanted > 100 * leak, $"wanted {wanted}, adjacent {leak}");
    }

    [Fact]
    public void Synchronous_detection_survives_a_faded_carrier()
    {
        // carrier faded 15 dB below the sidebands' reference: an envelope detector distorts badly,
        // a synchronous detector does not
        var faded = Am(Carrier, 1000, 0.5, 96000 * 2, carrierGain: 0.18);
        var env = new AmReceiver(Config(sync: false)).Process(faded);
        var syn = new AmReceiver(Config(sync: true)).Process(faded);
        double Thd(float[] a) => ToneAmplitude(a, 2000, 12000, 6000) / ToneAmplitude(a, 1000, 12000, 6000);
        Assert.True(Thd(syn) < 0.02, $"sync THD {Thd(syn)}");
        Assert.True(Thd(env) > 5 * Thd(syn), $"envelope THD {Thd(env)} vs sync {Thd(syn)}");
    }

    [Fact]
    public void Agc_levels_strong_and_weak_stations_alike()
    {
        var strong = new AmReceiver(Config(agc: true)).Process(Am(Carrier, 1000, 0.5, 96000 * 2, amp: 0.3));
        var weak = new AmReceiver(Config(agc: true)).Process(Am(Carrier, 1000, 0.5, 96000 * 2, amp: 0.003));
        double s = ToneAmplitude(strong, 1000, 12000, 12000), w = ToneAmplitude(weak, 1000, 12000, 12000);
        Assert.InRange(s / w, 0.8, 1.25);
    }

    [Theory]
    [InlineData(96000, 11000, 10000, 9000)]          // not an integer ratio
    [InlineData(96000, 12000, 10000, 12000)]         // channel wider than the audio rate allows
    [InlineData(96000, 12000, 40000, 9000)]          // channel outside the usable window
    public void Invalid_settings_are_rejected(double input, double audio, double tune, double bw)
    {
        Assert.Throws<ArgumentException>(() => new AmReceiver(new AmConfig(input, audio, tune, bw, false, false)));
    }

    [Fact]
    public void Both_receivers_share_the_demodulator_interface()
    {
        IDemodulator am = new AmReceiver(Config());
        IDemodulator ssb = new Receiver(new ReceiverConfig(Sideband.Upper, Fs, 12000, Carrier, 200, 2800, false));
        Assert.Equal(am.Process(new Complex[960]).Length, ssb.Process(new Complex[960]).Length);
    }
}
