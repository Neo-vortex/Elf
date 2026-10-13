using System.Globalization;
using NeoRadio.Dsp;

namespace NeoRadio.Dsp.Cli;

public sealed class CliUsageException(string message) : Exception(message);

public sealed record CliOptions(
    string InputPath,
    string OutputPath,
    string Mode,
    double TuneOffsetHz,
    double AudioRate,
    double LowCutHz,
    double HighCutHz,
    double CwPitchHz,
    double CwBandwidthHz,
    bool AgcEnabled,
    string? MetricsPath,
    double IqGainDb = 0,
    double IqPhaseDeg = 0,
    double AmBandwidthHz = 9000)
{
    public const string Usage =
        "usage: neoradio-dsp --input X.sigmf-meta --output Y.wav --tune-offset HZ [--mode usb|lsb|cw|am|sam] " +
        "[--audio-rate 12000] [--low-cut 200] [--high-cut 2800] [--cw-pitch 600] [--cw-bandwidth 500] " +
        "[--no-agc] [--metrics M.json] [--iq-gain-db 0] [--iq-phase-deg 0] [--am-bandwidth 9000]; " +
        "for am and sam the tune offset is the carrier's";

    public static CliOptions Parse(string[] args)
    {
        var values = new Dictionary<string, string>();
        bool agc = true;
        for (int i = 0; i < args.Length; i++)
        {
            string key = args[i];
            if (key == "--no-agc") { agc = false; continue; }
            if (key is not ("--input" or "--output" or "--mode" or "--tune-offset" or "--audio-rate" or "--low-cut"
                or "--high-cut" or "--cw-pitch" or "--cw-bandwidth" or "--metrics" or "--iq-gain-db" or "--iq-phase-deg"
                or "--am-bandwidth"))
                throw new CliUsageException($"unknown option {key}");
            if (i + 1 >= args.Length) throw new CliUsageException($"{key} needs a value");
            values[key] = args[++i];
        }

        string Required(string key) =>
            values.TryGetValue(key, out var v) ? v : throw new CliUsageException($"missing required option {key}");
        double Number(string key, double fallback)
        {
            if (!values.TryGetValue(key, out var v)) return fallback;
            if (!double.TryParse(v, NumberStyles.Float, CultureInfo.InvariantCulture, out var d))
                throw new CliUsageException($"{key} expects a number, got '{v}'");
            return double.IsFinite(d) ? d : throw new CliUsageException($"{key} must be a finite number, got '{v}'");
        }

        string mode = values.GetValueOrDefault("--mode", "usb");
        if (mode is not ("usb" or "lsb" or "cw" or "am" or "sam"))
            throw new CliUsageException($"--mode must be usb, lsb, cw, am or sam, got '{mode}'");

        double iqGain = Number("--iq-gain-db", 0), iqPhase = Number("--iq-phase-deg", 0);
        if (Math.Abs(iqGain) > IqCorrector.MaxGainDb)
            throw new CliUsageException($"--iq-gain-db must be within ±{IqCorrector.MaxGainDb} dB, got {iqGain}");
        if (Math.Abs(iqPhase) > IqCorrector.MaxPhaseDeg)
            throw new CliUsageException($"--iq-phase-deg must be within ±{IqCorrector.MaxPhaseDeg}°, got {iqPhase}");

        return new CliOptions(
            Required("--input"),
            Required("--output"),
            mode,
            values.ContainsKey("--tune-offset") ? Number("--tune-offset", 0) : throw new CliUsageException("missing required option --tune-offset"),
            Number("--audio-rate", 12000),
            Number("--low-cut", 200),
            Number("--high-cut", 2800),
            Number("--cw-pitch", 600),
            Number("--cw-bandwidth", 500),
            agc,
            values.GetValueOrDefault("--metrics"),
            iqGain,
            iqPhase,
            Number("--am-bandwidth", 9000));
    }

    /// <summary>The demodulator for the mode: AM (envelope or synchronous) or a single-sideband receiver.</summary>
    public IDemodulator ToDemodulator(double inputRate) => Mode is "am" or "sam"
        ? new AmReceiver(new AmConfig(inputRate, AudioRate, TuneOffsetHz, AmBandwidthHz, Mode == "sam", AgcEnabled))
        : new Receiver(ToReceiverConfig(inputRate));

    public ReceiverConfig ToReceiverConfig(double inputRate) => Mode switch
    {
        "cw" => new ReceiverConfig(Sideband.Upper, inputRate, AudioRate, TuneOffsetHz,
            CwPitchHz - CwBandwidthHz / 2, CwPitchHz + CwBandwidthHz / 2, AgcEnabled),
        "lsb" => new ReceiverConfig(Sideband.Lower, inputRate, AudioRate, TuneOffsetHz, LowCutHz, HighCutHz, AgcEnabled),
        _ => new ReceiverConfig(Sideband.Upper, inputRate, AudioRate, TuneOffsetHz, LowCutHz, HighCutHz, AgcEnabled),
    };
}
