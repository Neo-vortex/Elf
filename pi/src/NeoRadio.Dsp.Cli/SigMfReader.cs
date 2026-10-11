using System.Numerics;
using System.Runtime.InteropServices;
using System.Text.Json;

namespace NeoRadio.Dsp.Cli;

public sealed record SigMfRecording(double SampleRate, double CenterFrequencyHz, Complex[] Samples);

/// <summary>Reads SigMF recordings with core:datatype cf32_le.</summary>
public static class SigMfReader
{
    private const string MetaSuffix = ".sigmf-meta";

    public static SigMfRecording Read(string metaPath)
    {
        if (!metaPath.EndsWith(MetaSuffix, StringComparison.Ordinal))
            throw new InvalidDataException($"input must be a {MetaSuffix} file: {metaPath}");
        if (!File.Exists(metaPath))
            throw new FileNotFoundException($"metadata file not found: {metaPath}");

        using var doc = JsonDocument.Parse(File.ReadAllText(metaPath));
        if (!doc.RootElement.TryGetProperty("global", out var global))
            throw new InvalidDataException("metadata has no 'global' object");

        string datatype = global.TryGetProperty("core:datatype", out var dt) ? dt.GetString() ?? "" : "";
        if (datatype != "cf32_le")
            throw new InvalidDataException($"unsupported core:datatype '{datatype}', expected cf32_le");

        if (!global.TryGetProperty("core:sample_rate", out var sr) || sr.ValueKind != JsonValueKind.Number || sr.GetDouble() <= 0)
            throw new InvalidDataException("missing or invalid core:sample_rate");

        double centre = 0;
        if (doc.RootElement.TryGetProperty("captures", out var captures) && captures.ValueKind == JsonValueKind.Array
            && captures.GetArrayLength() > 0 && captures[0].TryGetProperty("core:frequency", out var f))
            centre = f.GetDouble();

        string dataPath = metaPath[..^MetaSuffix.Length] + ".sigmf-data";
        if (!File.Exists(dataPath)) throw new FileNotFoundException($"data file not found: {dataPath}");
        byte[] bytes = File.ReadAllBytes(dataPath);
        if (bytes.Length % 8 != 0)
            throw new InvalidDataException($"data file length {bytes.Length} is not a multiple of 8 bytes (cf32_le)");
        if (!BitConverter.IsLittleEndian)
            throw new PlatformNotSupportedException("big-endian hosts are not supported");

        var floats = MemoryMarshal.Cast<byte, float>(bytes);
        var samples = new Complex[floats.Length / 2];
        for (int i = 0; i < samples.Length; i++) samples[i] = new Complex(floats[2 * i], floats[2 * i + 1]);
        return new SigMfRecording(sr.GetDouble(), centre, samples);
    }
}
