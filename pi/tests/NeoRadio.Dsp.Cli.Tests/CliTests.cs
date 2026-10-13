using System.Numerics;
using System.Text;
using System.Text.Json;
using NeoRadio.Dsp.Cli;
using Xunit;

namespace NeoRadio.Dsp.Cli.Tests;

public sealed class CliTests : IDisposable
{
    private readonly string _dir = Directory.CreateTempSubdirectory("neoradio-cli-").FullName;

    public void Dispose() => Directory.Delete(_dir, recursive: true);

    static Complex[] Tone(double f, double fs, int n, double amp) =>
        Enumerable.Range(0, n).Select(i => Complex.FromPolarCoordinates(amp, 2 * Math.PI * f * i / fs)).ToArray();

    string WriteSigMf(string name, string datatype, double? rate, Complex[] samples, int dropBytes = 0)
    {
        var meta = Path.Combine(_dir, name + ".sigmf-meta");
        var global = new Dictionary<string, object> { ["core:datatype"] = datatype, ["core:version"] = "1.0.0" };
        if (rate is not null) global["core:sample_rate"] = rate.Value;
        var doc = new Dictionary<string, object>
        {
            ["global"] = global,
            ["captures"] = new[] { new Dictionary<string, object> { ["core:sample_start"] = 0, ["core:frequency"] = 14060000.0 } },
            ["annotations"] = Array.Empty<object>(),
        };
        File.WriteAllText(meta, JsonSerializer.Serialize(doc));
        using var ms = new MemoryStream();
        using (var w = new BinaryWriter(ms, Encoding.ASCII, leaveOpen: true))
            foreach (var s in samples) { w.Write((float)s.Real); w.Write((float)s.Imaginary); }
        var bytes = ms.ToArray();
        File.WriteAllBytes(Path.Combine(_dir, name + ".sigmf-data"), bytes[..^dropBytes]);
        return meta;
    }

    static (int Code, string Err) Run(params string[] args)
    {
        var err = new StringWriter();
        int code = CliApp.Run(args, err);
        return (code, err.ToString());
    }

    [Fact]
    public void Demodulates_to_wav_and_writes_metrics()
    {
        var meta = WriteSigMf("ok", "cf32_le", 96000, Tone(15500, 96000, 192000, 0.1));
        var wav = Path.Combine(_dir, "out.wav");
        var metrics = Path.Combine(_dir, "m.json");

        var (code, err) = Run("--input", meta, "--output", wav, "--mode", "usb", "--tune-offset", "14000", "--metrics", metrics);

        Assert.True(code == 0, err);
        var bytes = File.ReadAllBytes(wav);
        Assert.Equal("RIFF", Encoding.ASCII.GetString(bytes, 0, 4));
        Assert.Equal("WAVE", Encoding.ASCII.GetString(bytes, 8, 4));
        Assert.Equal(1, BitConverter.ToInt16(bytes, 22));       // channels
        Assert.Equal(12000, BitConverter.ToInt32(bytes, 24));   // sample rate
        Assert.Equal(16, BitConverter.ToInt16(bytes, 34));      // bits per sample
        Assert.Equal(24000 * 2, BitConverter.ToInt32(bytes, 40));
        using var m = JsonDocument.Parse(File.ReadAllText(metrics));
        Assert.Equal(192000, m.RootElement.GetProperty("input_samples").GetInt32());
        Assert.Equal(24000, m.RootElement.GetProperty("audio_samples").GetInt32());
        Assert.InRange(m.RootElement.GetProperty("audio_peak").GetDouble(), 0.2, 0.4);
    }

    [Fact]
    public void Unsupported_datatype_is_input_error()
    {
        var meta = WriteSigMf("ci16", "ci16_le", 96000, Tone(0, 96000, 10, 0.1));
        var (code, err) = Run("--input", meta, "--output", Path.Combine(_dir, "x.wav"), "--tune-offset", "0");
        Assert.Equal(3, code);
        Assert.Contains("cf32_le", err);
    }

    [Fact]
    public void Missing_sample_rate_is_input_error()
    {
        var meta = WriteSigMf("norate", "cf32_le", null, Tone(0, 96000, 10, 0.1));
        var (code, err) = Run("--input", meta, "--output", Path.Combine(_dir, "x.wav"), "--tune-offset", "0");
        Assert.Equal(3, code);
        Assert.Contains("core:sample_rate", err);
    }

    [Fact]
    public void Missing_data_file_is_input_error()
    {
        var meta = WriteSigMf("nodata", "cf32_le", 96000, Tone(0, 96000, 10, 0.1));
        File.Delete(Path.Combine(_dir, "nodata.sigmf-data"));
        var (code, err) = Run("--input", meta, "--output", Path.Combine(_dir, "x.wav"), "--tune-offset", "0");
        Assert.Equal(3, code);
        Assert.Contains("data file not found", err);
    }

    [Fact]
    public void Truncated_data_is_input_error()
    {
        var meta = WriteSigMf("trunc", "cf32_le", 96000, Tone(0, 96000, 10, 0.1), dropBytes: 3);
        var (code, err) = Run("--input", meta, "--output", Path.Combine(_dir, "x.wav"), "--tune-offset", "0");
        Assert.Equal(3, code);
        Assert.Contains("multiple of 8", err);
    }

    [Fact]
    public void Tune_offset_outside_window_is_usage_error()
    {
        var meta = WriteSigMf("far", "cf32_le", 96000, Tone(0, 96000, 96000, 0.1));
        var (code, err) = Run("--input", meta, "--output", Path.Combine(_dir, "x.wav"), "--tune-offset", "45000");
        Assert.Equal(2, code);
        Assert.Contains("outside the usable window", err);
    }

    [Fact]
    public void Unknown_option_is_usage_error()
    {
        var (code, err) = Run("--input", "a.sigmf-meta", "--output", "b.wav", "--tune-offset", "0", "--bogus", "1");
        Assert.Equal(2, code);
        Assert.Contains("unknown option --bogus", err);
    }

    [Fact]
    public void Missing_tune_offset_is_usage_error()
    {
        var (code, err) = Run("--input", "a.sigmf-meta", "--output", "b.wav");
        Assert.Equal(2, code);
        Assert.Contains("--tune-offset", err);
    }

    [Fact]
    public void Cw_mode_centres_passband_on_pitch()
    {
        var opts = CliOptions.Parse(new[] { "--input", "a.sigmf-meta", "--output", "b.wav", "--tune-offset", "0", "--mode", "cw" });
        var cfg = opts.ToReceiverConfig(96000);
        Assert.Equal(350, cfg.LowCutHz);
        Assert.Equal(850, cfg.HighCutHz);
        Assert.Equal(NeoRadio.Dsp.Sideband.Upper, cfg.Sideband);
    }

    string WriteRawMeta(string name, string json, int dataBytes = 16)
    {
        var meta = Path.Combine(_dir, name + ".sigmf-meta");
        File.WriteAllText(meta, json);
        File.WriteAllBytes(Path.Combine(_dir, name + ".sigmf-data"), new byte[dataBytes]);
        return meta;
    }

    [Theory]
    [InlineData("{\"global\":{\"core:datatype\":5,\"core:sample_rate\":96000}}", "core:datatype")]
    [InlineData("[]", "global")]
    [InlineData("{\"global\":{\"core:datatype\":\"cf32_le\",\"core:sample_rate\":96000},\"captures\":[{\"core:frequency\":\"x\"}]}", "core:frequency")]
    public void Malformed_metadata_is_input_error(string json, string expected)
    {
        var meta = WriteRawMeta("bad", json);
        var (code, err) = Run("--input", meta, "--output", Path.Combine(_dir, "x.wav"), "--tune-offset", "0");
        Assert.Equal(3, code);
        Assert.Contains(expected, err);
    }

    [Fact]
    public void Empty_data_file_is_input_error()
    {
        var meta = WriteRawMeta("empty", "{\"global\":{\"core:datatype\":\"cf32_le\",\"core:sample_rate\":96000}}", 0);
        var (code, err) = Run("--input", meta, "--output", Path.Combine(_dir, "x.wav"), "--tune-offset", "0");
        Assert.Equal(3, code);
        Assert.Contains("empty", err);
    }

    [Theory]
    [InlineData("--tune-offset", "NaN")]
    [InlineData("--low-cut", "NaN")]
    [InlineData("--high-cut", "Infinity")]
    public void Non_finite_numbers_are_usage_errors(string option, string value)
    {
        var args = new List<string> { "--input", "a.sigmf-meta", "--output", "b.wav", "--tune-offset", "0" };
        if (option == "--tune-offset") args[5] = value; else args.AddRange(new[] { option, value });
        var (code, err) = Run(args.ToArray());
        Assert.Equal(2, code);
        Assert.Contains("finite", err);
    }

    [Fact]
    public void Missing_output_directory_is_output_error()
    {
        var meta = WriteSigMf("okdir", "cf32_le", 96000, Tone(15500, 96000, 9600, 0.1));
        var (code, err) = Run("--input", meta, "--output", Path.Combine(_dir, "no", "such", "x.wav"), "--tune-offset", "14000");
        Assert.Equal(3, code);
        Assert.Contains("output error", err);
    }

    static Complex[] Imbalanced(Complex[] z, double gainDb, double phaseDeg)
    {
        double g = Math.Pow(10, gainDb / 20);
        var rot = Complex.FromPolarCoordinates(1, phaseDeg * Math.PI / 180);
        return z.Select(s => new Complex(s.Real, (s * rot).Imaginary / g)).ToArray();
    }

    /// <summary>RMS of a 16-bit mono WAV after the filters have settled (skips the start-up transient).</summary>
    static double SteadyRms(string wav, int skip = 2000)
    {
        var bytes = File.ReadAllBytes(wav);
        var samples = Enumerable.Range(0, (bytes.Length - 44) / 2).Skip(skip)
            .Select(i => BitConverter.ToInt16(bytes, 44 + 2 * i) / 32768.0).ToArray();
        return Math.Sqrt(samples.Average(v => v * v));
    }

    [Fact]
    public void Iq_correction_removes_the_image_of_a_station_on_the_other_side()
    {
        // a strong station at −15.5 kHz; its image (−20 dB from a 3 dB / 10° imbalance) lands on
        // the USB channel at +15.5 kHz (tune 14 kHz, audio 1.5 kHz)
        var input = Imbalanced(Tone(-15500, 96000, 96000, 0.5), 3.0, 10.0);
        var meta = WriteSigMf("img", "cf32_le", 96000, input);
        double Rms(params string[] extra)
        {
            var wav = Path.Combine(_dir, $"o{extra.Length}.wav");
            var (code, err) = Run(new[] { "--input", meta, "--output", wav, "--tune-offset", "14000", "--no-agc" }
                .Concat(extra).ToArray());
            Assert.True(code == 0, err);
            return SteadyRms(wav);
        }
        double raw = Rms();
        double corrected = Rms("--iq-gain-db", "3", "--iq-phase-deg", "10");
        Assert.True(raw > 0.01, $"image before correction {raw}");
        Assert.True(corrected < raw / 1000, $"image after correction {corrected} vs {raw}");   // > 60 dB
    }

    [Theory]
    [InlineData("--iq-gain-db", "7")]
    [InlineData("--iq-phase-deg", "-45")]
    [InlineData("--iq-gain-db", "NaN")]
    public void Iq_imbalance_out_of_range_is_a_usage_error(string option, string value)
    {
        var (code, err) = Run("--input", "a.sigmf-meta", "--output", "b.wav", "--tune-offset", "0", option, value);
        Assert.Equal(2, code);
        Assert.Contains(option, err);
    }

    [Fact]
    public void Iq_options_default_to_no_correction()
    {
        var o = CliOptions.Parse(new[] { "--input", "a", "--output", "b", "--tune-offset", "0" });
        Assert.Equal(0.0, o.IqGainDb);
        Assert.Equal(0.0, o.IqPhaseDeg);
        var p = CliOptions.Parse(new[] { "--input", "a", "--output", "b", "--tune-offset", "0", "--iq-gain-db", "0.16", "--iq-phase-deg", "-0.43" });
        Assert.Equal(0.16, p.IqGainDb);
        Assert.Equal(-0.43, p.IqPhaseDeg);
    }

    static Complex[] AmStation(double carrier, double tone, double m, int n, double amp = 0.1) =>
        Enumerable.Range(0, n).Select(i =>
            Complex.FromPolarCoordinates(amp * (1 + m * Math.Cos(2 * Math.PI * tone * i / 96000.0)),
                                         2 * Math.PI * carrier * i / 96000.0)).ToArray();

    [Theory]
    [InlineData("am")]
    [InlineData("sam")]
    public void Am_modes_demodulate_a_broadcast_station(string mode)
    {
        var meta = WriteSigMf("am", "cf32_le", 96000, AmStation(10000, 1000, 0.5, 192000));
        var wav = Path.Combine(_dir, "am.wav");
        var (code, err) = Run("--input", meta, "--output", wav, "--mode", mode, "--tune-offset", "10000", "--no-agc");
        Assert.True(code == 0, err);
        var bytes = File.ReadAllBytes(wav);
        var audio = Enumerable.Range(0, (bytes.Length - 44) / 2).Select(k => BitConverter.ToInt16(bytes, 44 + 2 * k) / 32768.0).ToArray();
        Complex acc = Complex.Zero;
        for (int k = 6000; k < audio.Length; k++) acc += audio[k] * Complex.FromPolarCoordinates(1, -2 * Math.PI * 1000 * k / 12000);
        Assert.InRange(2 * acc.Magnitude / (audio.Length - 6000), 0.04, 0.06);       // m · amplitude
    }

    [Fact]
    public void Am_bandwidth_too_wide_for_the_audio_rate_is_usage_error()
    {
        var meta = WriteSigMf("amw", "cf32_le", 96000, AmStation(10000, 1000, 0.5, 9600));
        var (code, err) = Run("--input", meta, "--output", Path.Combine(_dir, "x.wav"), "--mode", "am",
                              "--tune-offset", "10000", "--am-bandwidth", "12000");
        Assert.Equal(CliApp.UsageError, code);
        Assert.Contains("bandwidth", err);
    }

    [Fact]
    public void Unknown_mode_lists_the_am_modes()
    {
        var (code, err) = Run("--input", "x", "--output", "y", "--mode", "fm", "--tune-offset", "0");
        Assert.Equal(CliApp.UsageError, code);
        Assert.Contains("am or sam", err);
    }
}
