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
}
