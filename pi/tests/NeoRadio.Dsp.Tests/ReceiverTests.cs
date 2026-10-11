using System.Numerics;
using NeoRadio.Dsp;
using Xunit;

namespace NeoRadio.Dsp.Tests;

public class ReceiverTests
{
    const double Fs = 96000;

    static double ToneAmplitude(float[] audio, double f, double fs, int skip)
    {
        Complex acc = Complex.Zero;
        int n = 0;
        for (int i = skip; i < audio.Length; i++, n++)
            acc += audio[i] * Complex.FromPolarCoordinates(1, -2 * Math.PI * f * i / fs);
        return 2 * acc.Magnitude / n;
    }

    static ReceiverConfig Usb(double tune, bool agc = false) => new(Sideband.Upper, Fs, 12000, tune, 200, 2800, agc);
    static ReceiverConfig Lsb(double tune) => new(Sideband.Lower, Fs, 12000, tune, 200, 2800, false);
    static ReceiverConfig Cw(double tune) => new(Sideband.Upper, Fs, 12000, tune, 350, 850, false);

    [Fact]
    public void Usb_station_appears_at_its_audio_frequency()
    {
        var audio = new Receiver(Usb(14000)).Process(FilterTests.Tone(14000 + 1500, Fs, 96000, 0.1));
        Assert.Equal(12000, audio.Length);
        double wanted = ToneAmplitude(audio, 1500, 12000, 2000);
        Assert.InRange(wanted, 0.09, 0.11);
        Assert.True(wanted > 100 * ToneAmplitude(audio, 1400, 12000, 2000));
    }

    [Fact]
    public void Usb_rejects_opposite_sideband_by_60_dB()
    {
        var wanted = new Receiver(Usb(14000)).Process(FilterTests.Tone(14000 + 1500, Fs, 96000, 0.1));
        var image = new Receiver(Usb(14000)).Process(FilterTests.Tone(14000 - 1500, Fs, 96000, 0.1));
        double w = ToneAmplitude(wanted, 1500, 12000, 2000);
        double rms = Math.Sqrt(image.Skip(2000).Average(v => (double)v * v));
        Assert.True(rms < w * 1e-3, $"image rms {rms} vs wanted {w}");
    }

    [Fact]
    public void Lsb_station_below_dial_appears_at_its_audio_frequency()
    {
        var audio = new Receiver(Lsb(-20000)).Process(FilterTests.Tone(-20000 - 1000, Fs, 96000, 0.1));
        Assert.InRange(ToneAmplitude(audio, 1000, 12000, 2000), 0.09, 0.11);
    }

    [Fact]
    public void Cw_filter_passes_pitch_and_rejects_neighbour()
    {
        var on = new Receiver(Cw(5000)).Process(FilterTests.Tone(5600, Fs, 96000, 0.1));
        var off = new Receiver(Cw(5000)).Process(FilterTests.Tone(6200, Fs, 96000, 0.1));
        double a = ToneAmplitude(on, 600, 12000, 2000);
        double rms = Math.Sqrt(off.Skip(2000).Average(v => (double)v * v));
        Assert.InRange(a, 0.09, 0.11);
        Assert.True(rms < a * 1e-3);
    }

    [Fact]
    public void Streaming_matches_one_shot()
    {
        var x = FilterTests.Tone(15500, Fs, 48000, 0.1);
        var once = new Receiver(Usb(14000)).Process(x);
        var rx = new Receiver(Usb(14000));
        var parts = rx.Process(x.AsSpan(0, 20003)).Concat(rx.Process(x.AsSpan(20003))).ToArray();
        Assert.Equal(once.Length, parts.Length);
        for (int i = 0; i < once.Length; i++) Assert.Equal(once[i], parts[i], 5);
    }

    [Fact]
    public void Agc_enabled_normalises_weak_signal()
    {
        var audio = new Receiver(Usb(14000, agc: true)).Process(FilterTests.Tone(15500, Fs, 96000, 1e-4));
        Assert.InRange(audio.Skip(6000).Max(v => Math.Abs(v)), 0.25f, 0.35f);
    }

    [Fact]
    public void Config_validation_rejects_tune_outside_window() =>
        Assert.Throws<ArgumentException>(() => new Receiver(Usb(42000)));

    [Fact]
    public void Config_validation_rejects_non_integer_rate_ratio() =>
        Assert.Throws<ArgumentException>(() => new Receiver(new ReceiverConfig(Sideband.Upper, Fs, 11025, 0, 200, 2800)));

    [Fact]
    public void Config_validation_rejects_inverted_passband() =>
        Assert.Throws<ArgumentException>(() => new Receiver(new ReceiverConfig(Sideband.Upper, Fs, 12000, 0, 2800, 200)));

    [Fact]
    public void Config_validation_rejects_passband_above_audio_nyquist() =>
        Assert.Throws<ArgumentException>(() => new Receiver(new ReceiverConfig(Sideband.Upper, Fs, 12000, 0, 200, 6000)));
}
